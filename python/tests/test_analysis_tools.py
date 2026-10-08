"""
Tests of the analysis tools (code design Section 9): simulate, register, residual_maps, compare, report.

Every session is synthetic. The camera is 320 x 240 (the simulator's default) or 160 x 120 for the unit
tests of the simulator itself, so the whole file runs in well under a minute and a half. Sessions that several
tests share are built once per module by the ``analysis_*`` fixtures.
"""
from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy import ndimage

from planereg.analysis import compare, register, report, residual_maps, simulate
from sphcal.cli.make_manifest import Messages, read_pose_log
from sphcal.geometry.transforms import RigidTransform
from sphcal.io.matcloud import read_matcloud

ROTATION_TOLERANCE_DEG = 0.05
TRANSLATION_TOLERANCE_MM = 0.5
"""Acceptance of the recovered transform (design Section 9)."""
SCALE_TOLERANCE = 1e-3
"""Acceptance of the recovered scale (design Section 9)."""
INJECTED_SCALE = 1.02
"""The scale injected into the scaled session."""
BACKLASH_DEG = 0.3
BACKLASH_MM = 0.5
"""Backlash emulation of the comparison sessions."""
SMALL_IMAGE_SIZE = (160, 120)
"""Image size of the simulator unit tests."""
DEFAULT_POSE_COUNT_RANGE = (20, 28)
"""The default plan makes about 24 poses."""
FLYAWAY_FRACTION_TOLERANCE = 0.05
"""Allowed deviation of the displaced fraction of the silhouette ring from the requested one."""
SURFACE_RGB = (252, 252, 251)
"""#fcfcfb as 8-bit RGB: the figure background."""
FLOAT32_RELATIVE_TOLERANCE = 1e-6
"""Relative agreement of float32 depth images."""
BOARD_MASK_SIZE_PX = (40, 60)
"""Rows and columns of the synthetic board mask of the fly-away test."""
IMAGE_DEPTH_MM = 600.0
"""Depth of the board in the clutter test."""
HELD_OUT_EVERY = 4
"""Every fourth pose of the holdout session is tagged held out in its plan (6 of 24)."""
HELD_OUT_RMS_NORMAL_TOLERANCE_DEG = 0.1
HELD_OUT_RMS_OFFSET_TOLERANCE_MM = 0.5
"""On clean synthetic data the held-out poses agree with the transform fitted without them to within the noise of
the segmentation (the fit's own residuals are of this size)."""
FACING_ROTATION = np.diag([1.0, -1.0, -1.0])
"""Board-to-sensor rotation of a board that faces the sensor head on (its z axis points back at the sensor)."""


# ---------------------------------------------------------------------------
# Helpers and fixtures
# ---------------------------------------------------------------------------
def rigid_from_truth(truth: dict) -> RigidTransform:
    """The true rigid sensor-to-base transform of a truth.json dictionary."""
    entry = truth["sensor_to_base"]
    return RigidTransform(np.array(entry["rotation"]), np.array(entry["translation_mm"]))


def final_errors(block: dict, truth: dict) -> tuple[float, float]:
    """(rotation error deg, translation error mm) of a result block against the truth, computed here
    independently of the tool under test."""
    estimated = RigidTransform(np.array(block["rotation"]), np.array(block["translation_mm"]))
    translation, rotation = estimated.difference_from(rigid_from_truth(truth))
    return rotation, translation


def write_rough(truth: dict, path: Path) -> Path:
    """A --sensor-in-base file holding the true rigid transform of a session."""
    path.write_text(json.dumps({"matrix": rigid_from_truth(truth).as_matrix().reshape(-1).tolist()}), encoding="utf-8")
    return path


def make_session(directory: Path, *extra: str) -> dict:
    """Run the simulator CLI into ``directory`` and return its truth."""
    assert simulate.main(["--out", str(directory), *extra]) == simulate.EXIT_OK
    return json.loads((directory / "truth.json").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def analysis_clean(tmp_path_factory):
    """A default-plan session with clutter and fly-aways, registered with both models and figures."""
    root = tmp_path_factory.mktemp("analysis_clean")
    truth = make_session(root / "session", "--clutter", "--flyaways", "--frames", "2", "--seed", "1")
    exit_code = register.main(["--manifest", str(root / "session" / "manifest.json"), "--out", str(root / "reg"),
                               "--truth", str(root / "session" / "truth.json"), "--pixel-maps", "2",
                               "--pixel-maps-random", "1"])
    document = json.loads((root / "reg" / "registration.json").read_text(encoding="utf-8"))
    return {"root": root, "truth": truth, "exit": exit_code, "document": document}


@pytest.fixture(scope="module")
def analysis_scaled(tmp_path_factory):
    """A session whose sensor reports lengths in a unit 1 / 1.02 of the base's, with clutter and fly-aways."""
    root = tmp_path_factory.mktemp("analysis_scaled")
    truth = make_session(root / "session", "--clutter", "--flyaways", "--frames", "2", "--seed", "2",
                         "--scale", str(INJECTED_SCALE))
    register.main(["--manifest", str(root / "session" / "manifest.json"), "--out", str(root / "reg"),
                   "--no-figures"])
    document = json.loads((root / "reg" / "registration.json").read_text(encoding="utf-8"))
    return {"root": root, "truth": truth, "document": document}


@pytest.fixture(scope="module")
def analysis_backlash(tmp_path_factory):
    """Procedures A and B of one plan and sensor placement with backlash emulation, registered and compared."""
    root = tmp_path_factory.mktemp("analysis_backlash")
    for procedure in (simulate.PROCEDURE_A, simulate.PROCEDURE_B):
        truth = make_session(root / procedure, "--procedure", procedure, "--backlash-deg", str(BACKLASH_DEG),
                             "--backlash-mm", str(BACKLASH_MM), "--frames", "2", "--seed", "4")
        rough = write_rough(truth, root / f"rough_{procedure}.json")
        register.main(["--manifest", str(root / procedure / "manifest.json"), "--out", str(root / f"reg_{procedure}"),
                       "--sensor-in-base", str(rough), "--model", "rigid", "--no-figures"])
    exit_code = compare.main(["--registration-a", str(root / "reg_A" / "registration.json"),
                              "--registration-b", str(root / "reg_B" / "registration.json"),
                              "--out", str(root / "cmp")])
    comparison = json.loads((root / "cmp" / "comparison.json").read_text(encoding="utf-8"))
    return {"root": root, "exit": exit_code, "comparison": comparison}


# ---------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------
def test_simulator_writes_a_session_in_the_layout_of_a_real_one(analysis_clean):
    session = analysis_clean["root"] / "session"
    truth = analysis_clean["truth"]
    low, high = DEFAULT_POSE_COUNT_RANGE
    assert low <= len(truth["poses"]) <= high
    files = sorted((session / "captures").glob("*.mc"))
    assert len(files) == 2 * len(truth["poses"])
    assert files[0].name.endswith("_Index00.mc")
    capture = read_matcloud(files[0])
    assert {"fx", "fy", "cx", "cy", "h", "v", "cameraName", "version", "robotPose"} <= set(capture.header)
    assert capture.matrices["XYZ"].dtype == np.float32
    # The technician's pose log is accepted by sphcal's own reader, one row per pose.
    messages = Messages()
    logged = read_pose_log(session / "pose_log.csv", messages)
    assert not messages.errors and len(logged) == len(truth["poses"])


def test_simulator_reported_pose_in_headers_true_pose_in_rendering_and_backlash_signs(tmp_path):
    params = simulate.SimulationParameters(image_width_px=SMALL_IMAGE_SIZE[0], image_height_px=SMALL_IMAGE_SIZE[1],
                                           frames_per_pose=1, backlash_deg=BACKLASH_DEG, backlash_mm=BACKLASH_MM)
    truth_a = simulate.simulate_session(tmp_path / "a", params)
    truth_b = simulate.simulate_session(tmp_path / "b", replace(params, procedure=simulate.PROCEDURE_B))
    signs_a = {pose["backlash_sign"] for pose in truth_a["poses"]}
    signs_b = {pose["backlash_sign"] for pose in truth_b["poses"]}
    assert signs_a == {-1.0, 1.0}                       # random sign per pose in A
    assert signs_b == {simulate.CONSTANT_BACKLASH_SIGN}  # constant sign in B
    # Same sensor placement in A and B of one seed, and the same reported poses.
    assert truth_a["sensor_to_base"] == truth_b["sensor_to_base"]
    for pose in truth_a["poses"]:
        reported = RigidTransform.from_matrix(np.array(pose["reported_pose"]).reshape(4, 4))
        true = RigidTransform.from_matrix(np.array(pose["true_pose"]).reshape(4, 4))
        translation, rotation = true.difference_from(reported)
        assert translation == pytest.approx(BACKLASH_MM, abs=1e-9)
        assert rotation == pytest.approx(BACKLASH_DEG, abs=1e-9)
        # The capture header carries the REPORTED pose.
        header = read_matcloud(next((tmp_path / "a" / "captures").glob(f"{pose['pose_id']}_Index00.mc"))).header
        assert np.allclose(header["robotPose"], pose["reported_pose"])


def test_simulator_scale_is_a_length_unit_factor_on_the_rendered_xyz():
    params = simulate.SimulationParameters(image_width_px=SMALL_IMAGE_SIZE[0], image_height_px=SMALL_IMAGE_SIZE[1])
    scaled = replace(params, scale=INJECTED_SCALE)
    camera = simulate.camera_from_parameters(params)
    sensor = simulate.sensor_parameters(params, camera)
    board_to_sensor = RigidTransform(FACING_ROTATION, np.array([0.0, 0.0, IMAGE_DEPTH_MM]))
    frames = [simulate.render_frame(sensor, board_to_sensor, params.board_half_size_mm, p, simulate.zero_error_field,
                                    np.random.default_rng(0)) for p in (params, scaled)]
    valid = frames[0][..., 2] > 0.0
    assert valid.any() and np.array_equal(valid, frames[1][..., 2] > 0.0)
    assert np.allclose(frames[1][valid], frames[0][valid] / INJECTED_SCALE, rtol=FLOAT32_RELATIVE_TOLERANCE)


def test_simulator_flyaways_move_only_the_silhouette_ring_by_the_requested_factors():
    params = simulate.SimulationParameters(image_width_px=SMALL_IMAGE_SIZE[0], image_height_px=SMALL_IMAGE_SIZE[1])
    camera = simulate.camera_from_parameters(params)
    xyz = (camera.ray_directions() * IMAGE_DEPTH_MM).astype(np.float32)
    original = xyz.copy()
    mask = np.zeros(xyz.shape[:2], dtype=bool)
    rows, columns = BOARD_MASK_SIZE_PX
    mask[camera.height // 2 - rows // 2:camera.height // 2 + rows // 2,
         camera.width // 2 - columns // 2:camera.width // 2 + columns // 2] = True
    count = simulate.apply_flyaways(xyz, mask, params, np.random.default_rng(0))
    moved = np.any(xyz != original, axis=-1)
    assert count == int(moved.sum()) > 0
    eroded = ndimage.binary_erosion(mask, structure=ndimage.generate_binary_structure(2, 1),
                                    iterations=params.flyaway_band_px, border_value=1)
    ring = mask & ~eroded
    assert not (moved & ~ring).any()                    # nothing outside the ring moved
    assert moved.sum() / ring.sum() == pytest.approx(params.flyaway_fraction, abs=FLYAWAY_FRACTION_TOLERANCE)
    factor = xyz[moved, 2] / original[moved, 2]
    low, high = params.flyaway_factor_range
    assert factor.min() >= low - FLOAT32_RELATIVE_TOLERANCE and factor.max() <= high + FLOAT32_RELATIVE_TOLERANCE
    # Moved along the ray: the direction of every moved point is unchanged.
    assert np.allclose(xyz[moved] / np.linalg.norm(xyz[moved], axis=-1, keepdims=True),
                       original[moved] / np.linalg.norm(original[moved], axis=-1, keepdims=True), atol=1e-5)


def test_simulator_clutter_is_behind_the_board_and_the_board_wins_where_it_is_in_front():
    params = simulate.SimulationParameters(image_width_px=SMALL_IMAGE_SIZE[0], image_height_px=SMALL_IMAGE_SIZE[1],
                                           clutter=True)
    camera = simulate.camera_from_parameters(params)
    sensor = simulate.sensor_parameters(params, camera)
    board_to_sensor = RigidTransform(FACING_ROTATION, np.array([0.0, 0.0, IMAGE_DEPTH_MM]))
    frame = simulate.render_frame(sensor, board_to_sensor, params.board_half_size_mm, params, simulate.zero_error_field,
                                  np.random.default_rng(0))
    z = frame[..., 2]
    center = z[camera.height // 2, camera.width // 2]
    assert center == pytest.approx(IMAGE_DEPTH_MM, abs=1.0)            # the board in front
    corner = z[0, 0]
    assert corner == 0.0 or corner > IMAGE_DEPTH_MM + 0.5 * params.wall_distance_behind_mm   # the wall behind it
    wall_depths = z[z > IMAGE_DEPTH_MM + 1.0]
    assert wall_depths.size > 0 and wall_depths.min() >= IMAGE_DEPTH_MM


def test_simulator_input_errors_exit_two(tmp_path):
    assert simulate.main(["--out", str(tmp_path / "x"), "--plan", str(tmp_path / "missing.csv")]) == \
        simulate.EXIT_INPUT_ERROR
    assert simulate.main(["--out", str(tmp_path / "y"), "--scale", "-1"]) == simulate.EXIT_INPUT_ERROR


def test_simulator_accepts_a_plan_csv_with_the_capture_tools_extra_columns(tmp_path):
    session = tmp_path / "default"
    truth = make_session(session, "--frames", "1", "--seed", "5", "--image-size", *map(str, SMALL_IMAGE_SIZE))
    # Rebuild a plan CSV from the first two poses, with extra columns like the capture tools' plan adds.
    plan = tmp_path / "poses.csv"
    columns = ([simulate.PLAN_POSE_ID_COLUMN] + ["kind", "half_width_mm", "half_height_mm"]
               + list(simulate.PLAN_POSITION_COLUMNS) + list(simulate.PLAN_ROTATION_COLUMNS)
               + ["standoff_mm", "approach_x_mm"])
    lines = [",".join(columns)]
    for pose in truth["poses"][:2]:
        matrix = np.array(pose["reported_pose"]).reshape(4, 4)
        lines.append(",".join([pose["pose_id"], "board", *map(str, pose["half_size_mm"]),
                               *map(str, matrix[:3, 3]), *map(str, matrix[:3, :3].reshape(-1)), "450", "0"]))
    plan.write_text("\n".join(lines), encoding="utf-8")
    rough = write_rough(truth, tmp_path / "rough.json")
    assert simulate.main(["--out", str(tmp_path / "from_plan"), "--plan", str(plan), "--sensor-to-base", str(rough),
                          "--frames", "1", "--image-size", *map(str, SMALL_IMAGE_SIZE)]) == simulate.EXIT_OK
    replayed = json.loads((tmp_path / "from_plan" / "truth.json").read_text(encoding="utf-8"))
    assert [p["pose_id"] for p in replayed["poses"]] == [p["pose_id"] for p in truth["poses"][:2]]
    assert np.allclose(replayed["poses"][0]["reported_pose"], truth["poses"][0]["reported_pose"])


# ---------------------------------------------------------------------------
# register
# ---------------------------------------------------------------------------
def test_register_recovers_the_transform_in_both_passes_and_models(analysis_clean):
    document, truth = analysis_clean["document"], analysis_clean["truth"]
    assert analysis_clean["exit"] == register.EXIT_OK
    assert document["model_order"] == ["rigid", "similarity"]
    for model in document["model_order"]:
        entry = document["models"][model]
        assert entry["final_pass"] == register.PASS_TWO
        for pass_name in (register.PASS_ONE, register.PASS_TWO):
            block = entry[pass_name]
            assert block["accepted"] and block["poses_used"] == block["poses_total"] == len(truth["poses"])
            rotation, translation = final_errors(block, truth)
            assert rotation < ROTATION_TOLERANCE_DEG, (model, pass_name)
            assert translation < TRANSLATION_TOLERANCE_MM, (model, pass_name)
    # Pass 2 segments every pose by the predicted region of the pass-1 transform.
    methods = {pose["method"] for pose in document["models"]["rigid"][register.PASS_TWO]["poses"]}
    assert methods == {"predicted_region"}
    # The tool's own comparison with the truth agrees with the independent computation above.
    assert document["truth_comparison"]["rigid"][register.PASS_TWO]["rotation_error_deg"] < ROTATION_TOLERANCE_DEG


def test_register_with_a_rough_transform_predicts_in_the_first_pass_too(analysis_clean, tmp_path):
    rough = write_rough(analysis_clean["truth"], tmp_path / "rough.json")
    out = tmp_path / "reg"
    assert register.main(["--manifest", str(analysis_clean["root"] / "session" / "manifest.json"), "--out", str(out),
                          "--sensor-in-base", str(rough), "--model", "rigid", "--no-figures"]) == register.EXIT_OK
    document = json.loads((out / "registration.json").read_text(encoding="utf-8"))
    assert document["model_order"] == ["rigid"]
    assert document["models"]["rigid"][register.PASS_ONE]["prediction"] == register.PREDICTION_SENSOR_IN_BASE
    assert {p["method"] for p in document["models"]["rigid"][register.PASS_ONE]["poses"]} == {"predicted_region"}


def test_register_writes_masks_keyed_by_model_pass_and_pose(analysis_clean):
    masks = np.load(analysis_clean["root"] / "reg" / "segmentation.npz")
    pose_id = analysis_clean["truth"]["poses"][0]["pose_id"]
    for model in ("rigid", "similarity"):
        for pass_name in (register.PASS_ONE, register.PASS_TWO):
            mask = masks[register.MASK_KEY_FORMAT.format(model=model, pass_name=pass_name, pose_id=pose_id)]
            assert mask.dtype == bool and mask.sum() > 0
    assert masks[register.MASK_KEY_FORMAT.format(model="rigid", pass_name="pass2", pose_id=pose_id)].shape == \
        tuple(reversed(analysis_clean["document"]["image_size_px"]))


def test_similarity_model_recovers_an_injected_scale(analysis_scaled):
    document, truth = analysis_scaled["document"], analysis_scaled["truth"]
    assert truth["sensor_to_base"]["scale"] == INJECTED_SCALE
    for pass_name in (register.PASS_ONE, register.PASS_TWO):
        block = document["models"]["similarity"][pass_name]
        assert block["scale"] == pytest.approx(INJECTED_SCALE, abs=SCALE_TOLERANCE), pass_name
        rotation, translation = final_errors(block, truth)
        assert rotation < ROTATION_TOLERANCE_DEG and translation < TRANSLATION_TOLERANCE_MM
    # The rigid model cannot absorb a scale: its residuals are visibly larger.
    rigid = document["models"]["rigid"][register.PASS_TWO]
    similarity = document["models"]["similarity"][register.PASS_TWO]
    assert rigid["rms_offset_residual_mm"] > similarity["rms_offset_residual_mm"]


def test_register_reports_unreadable_manifests_as_input_errors(tmp_path):
    assert register.main(["--manifest", str(tmp_path / "missing.json"), "--out", str(tmp_path / "out")]) == \
        register.EXIT_INPUT_ERROR


# ---------------------------------------------------------------------------
# residual_maps
# ---------------------------------------------------------------------------
def test_figures_are_written_with_the_design_resolution_and_surface(analysis_clean):
    pytest.importorskip("matplotlib")
    import matplotlib.image as mpimg
    directory = analysis_clean["root"] / "reg" / register.FIGURE_DIRECTORY_NAME / "rigid"
    names = {path.name for path in directory.glob("*.png")}
    assert {residual_maps.POSE_RESIDUALS_FILE, residual_maps.NORMAL_FIELD_FILE,
            residual_maps.OFFSET_FIELD_FILE} <= names
    assert sum(name.startswith("pixel_residuals_") for name in names) == 3       # 2 largest + 1 random
    image = mpimg.imread(directory / residual_maps.POSE_RESIDUALS_FILE)
    assert tuple(np.round(image[0, 0, :3] * 255).astype(int)) == SURFACE_RGB
    # A 5 x 4.4 inch panel at 150 dpi (one column, two rows).
    params = residual_maps.FigureParameters()
    assert image.shape[1] == round(params.panel_width_in * residual_maps.FIGURE_DPI)
    assert image.shape[0] == round(2 * params.panel_height_in * residual_maps.FIGURE_DPI)


def test_residual_maps_command_line_redraws_from_the_files(analysis_clean, tmp_path):
    pytest.importorskip("matplotlib")
    assert residual_maps.main(["--registration", str(analysis_clean["root"] / "reg" / "registration.json"),
                               "--out", str(tmp_path), "--model", "rigid", "--pixel-maps", "1",
                               "--pixel-maps-random", "0"]) == residual_maps.EXIT_OK
    assert len(list(tmp_path.glob("*.png"))) == 4


def test_figures_degrade_with_a_warning_without_matplotlib(analysis_clean, tmp_path, monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "matplotlib", None)       # makes "import matplotlib" raise ImportError
    written = residual_maps.make_figures(analysis_clean["document"], {}, tmp_path, "rigid")
    assert written == [] and not list(tmp_path.iterdir())
    assert residual_maps.plot_comparison(tmp_path / "c.png", ("A", "B"), (np.ones(2), np.ones(2)),
                                         (np.ones(2), np.ones(2)), "title") is None
    assert "WARNING" in capsys.readouterr().out


def test_residual_maps_helpers_choose_standoff_groups_and_round_key_lengths():
    depths = np.array([452.0, 649.0, 447.0, 851.0, 655.0])
    assert residual_maps.group_standoffs(depths, 100.0).tolist() == [0, 1, 0, 2, 1]
    assert residual_maps.nice_length(0.043) == pytest.approx(0.02)
    assert residual_maps.nice_length(7.0) == pytest.approx(5.0)
    assert residual_maps.symmetric_limit(np.array([-0.3, 0.1, np.nan])) == pytest.approx(0.3)


# ---------------------------------------------------------------------------
# compare and report
# ---------------------------------------------------------------------------
def test_compare_reports_a_smaller_residual_rms_for_procedure_b(analysis_backlash):
    comparison = analysis_backlash["comparison"]
    assert analysis_backlash["exit"] == compare.EXIT_OK
    entry = comparison["models"]["rigid"]
    assert entry["b_smaller_rms"]
    assert entry["rms_normal_residual_deg"]["b"] < entry["rms_normal_residual_deg"]["a"]
    assert entry["rms_offset_residual_mm"]["b"] < entry["rms_offset_residual_mm"]["a"]
    assert entry["poses_common"] == entry["poses_used_a"] == entry["poses_used_b"]
    relative = entry["relative_transform"]
    assert relative["rotation_angle_deg"] > 0.0 and relative["translation_distance_mm"] > 0.0
    assert len(entry["pose_differences"]) == entry["poses_common"]
    if comparison["figure"] is not None:
        assert (analysis_backlash["root"] / "cmp" / comparison["figure"]).stat().st_size > 0


def test_compare_rejects_files_that_are_not_registrations(analysis_backlash, tmp_path):
    bogus = tmp_path / "registration.json"
    bogus.write_text("{}", encoding="utf-8")
    assert compare.main(["--registration-a", str(bogus), "--registration-b", str(bogus),
                         "--out", str(tmp_path / "out")]) == compare.EXIT_INPUT_ERROR


def test_report_contains_the_transform_the_verdict_and_the_comparison(analysis_clean, analysis_backlash, tmp_path):
    out = tmp_path / "report.md"
    assert report.main(["--registration", str(analysis_clean["root"] / "reg" / "registration.json"),
                        "--comparison", str(analysis_backlash["root"] / "cmp" / "comparison.json"),
                        "--out", str(out)]) == report.EXIT_OK
    text = out.read_text(encoding="utf-8")
    block = analysis_clean["document"]["models"]["rigid"]["pass2"]
    translation = block["translation_mm"]
    assert "### Transform (sensor to base)" in text
    assert f"{translation[0]:.{report.DIGITS}f}" in text
    assert "Verdict: ACCEPTED" in text
    assert "### Spreads of the pose set" in text and "similarity spread" in text
    assert "## Comparison of two sessions" in text
    assert "pose_residuals.png" in text                     # the figure list


# ---------------------------------------------------------------------------
# Command lines
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# Held-out poses
# ---------------------------------------------------------------------------
def write_plan_csv(path: Path, truth: dict, held_out_ids: set[str], omit: tuple[str, ...] = (),
                   extra_rows: tuple[tuple[str, int], ...] = (), holdout_column: bool = True) -> Path:
    """A plan CSV of the truth's poses in the capture tools' layout (the simulator's plan columns, then
    ``standoff_mm`` and the ``holdout`` tag), with the poses of ``omit`` left out and ``extra_rows``
    (pose id, tag) added for planned poses that were never captured."""
    columns = ([simulate.PLAN_POSE_ID_COLUMN, "kind", "half_width_mm", "half_height_mm"]
               + list(simulate.PLAN_POSITION_COLUMNS) + list(simulate.PLAN_ROTATION_COLUMNS) + ["standoff_mm"]
               + (["holdout"] if holdout_column else []))
    lines = [",".join(columns)]
    for pose in truth["poses"]:
        if pose["pose_id"] in omit:
            continue
        matrix = np.array(pose["reported_pose"]).reshape(4, 4)
        cells = [pose["pose_id"], "board", *map(str, pose["half_size_mm"]), *map(str, matrix[:3, 3]),
                 *map(str, matrix[:3, :3].reshape(-1)), "450"]
        lines.append(",".join(cells + ([str(int(pose["pose_id"] in held_out_ids))] if holdout_column else [])))
    template = lines[1].split(",")
    for pose_id, tag in extra_rows:
        lines.append(",".join([pose_id, *template[1:-1], str(tag)]))
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


@pytest.fixture(scope="module")
def analysis_holdout(tmp_path_factory):
    """A default-plan session on the small camera whose plan tags every HELD_OUT_EVERY-th pose as held out, and
    its registration (both models) with that plan."""
    root = tmp_path_factory.mktemp("analysis_holdout")
    first = make_session(root / "first", "--frames", "1", "--seed", "6", "--image-size", *map(str, SMALL_IMAGE_SIZE))
    held_out = {pose["pose_id"] for pose in first["poses"][HELD_OUT_EVERY - 1::HELD_OUT_EVERY]}
    plan = write_plan_csv(root / "poses.csv", first, held_out)
    rough = write_rough(first, root / "rough.json")
    # Replay the plan so that the session is made from a poses.csv with the plan's extra columns.
    truth = make_session(root / "session", "--plan", str(plan), "--sensor-to-base", str(rough), "--frames", "1",
                         "--seed", "6", "--image-size", *map(str, SMALL_IMAGE_SIZE))
    manifest = root / "session" / "manifest.json"
    exit_code = register.main(["--manifest", str(manifest), "--plan", str(plan), "--out", str(root / "reg"),
                               "--no-figures", "--truth", str(root / "session" / "truth.json")])
    document = json.loads((root / "reg" / "registration.json").read_text(encoding="utf-8"))
    return {"root": root, "truth": truth, "plan": plan, "manifest": manifest, "held_out": held_out,
            "exit": exit_code, "document": document}


def test_register_with_a_plan_fits_only_the_poses_that_are_not_held_out(analysis_holdout):
    document, truth, held_out = analysis_holdout["document"], analysis_holdout["truth"], analysis_holdout["held_out"]
    pose_ids = [pose["pose_id"] for pose in truth["poses"]]
    assert held_out and len(held_out) < len(pose_ids)
    assert analysis_holdout["exit"] == register.EXIT_OK
    for model in document["model_order"]:
        for pass_name in (register.PASS_ONE, register.PASS_TWO):
            block = document["models"][model][pass_name]
            # The fit saw exactly the poses that are not held out; the entries still list every pose.
            assert block["poses_total"] == len(pose_ids)
            assert block["poses_used"] == len(pose_ids) - len(held_out)
            assert [p["pose_id"] for p in block["poses"]] == pose_ids
            for pose in block["poses"]:
                assert pose["held_out"] == (pose["pose_id"] in held_out)
                assert pose["used"] == (pose["pose_id"] not in held_out)
            assert block["rejected_pose_ids"] == []
            # The held-out block lists exactly the tagged poses, with finite residuals.
            section = block["held_out"]
            assert section["count"] == len(held_out) and section["unevaluated_pose_ids"] == []
            assert {p["pose_id"] for p in section["poses"]} == held_out
            for entry in section["poses"]:
                assert np.isfinite(entry["normal_residual_deg"]) and np.isfinite(entry["offset_residual_mm"])
            assert 0.0 <= section["rms_normal_residual_deg"] < HELD_OUT_RMS_NORMAL_TOLERANCE_DEG
            assert 0.0 <= section["rms_offset_residual_mm"] < HELD_OUT_RMS_OFFSET_TOLERANCE_MM
            assert section["max_normal_residual_deg"] >= section["rms_normal_residual_deg"]
            assert section["max_offset_residual_mm"] >= section["rms_offset_residual_mm"]
            assert section["held_out_within_limits"] is True
            # The per-pose entries of a held-out pose carry the same residuals as the block.
            by_id = {p["pose_id"]: p for p in block["poses"]}
            for entry in section["poses"]:
                assert by_id[entry["pose_id"]]["normal_residual_deg"] == entry["normal_residual_deg"]
                assert by_id[entry["pose_id"]]["offset_residual_mm"] == entry["offset_residual_mm"]
    rotation, translation = final_errors(document["models"]["rigid"][register.PASS_TWO], truth)
    assert rotation < ROTATION_TOLERANCE_DEG and translation < TRANSLATION_TOLERANCE_MM
    assert document["plan"]["poses_not_captured"] == [] and document["plan"]["manifest_poses_not_in_plan"] == []


def test_register_without_a_plan_holds_nothing_out(analysis_holdout, tmp_path):
    assert register.main(["--manifest", str(analysis_holdout["manifest"]), "--out", str(tmp_path), "--model", "rigid",
                          "--no-figures"]) == register.EXIT_OK
    document = json.loads((tmp_path / "registration.json").read_text(encoding="utf-8"))
    block = document["models"]["rigid"][register.PASS_TWO]
    assert block["held_out"] is None and document["plan"] is None
    assert block["poses_used"] == block["poses_total"] and not any(p["held_out"] for p in block["poses"])


def test_register_prints_a_held_out_line_per_model_and_exits_one_when_they_exceed_the_limits(analysis_holdout, tmp_path,
                                                                                                capsys):
    arguments = ["--manifest", str(analysis_holdout["manifest"]), "--plan", str(analysis_holdout["plan"]),
                 "--out", str(tmp_path), "--no-figures"]
    assert register.main(arguments) == register.EXIT_OK
    lines = [line for line in capsys.readouterr().out.splitlines() if "held out:" in line]
    assert len(lines) == 2 and lines[0].startswith("rigid") and lines[1].startswith("similarity")
    count = len(analysis_holdout["held_out"])
    assert all(f"held out: {count} poses, RMS normal " in line and line.endswith("within limits") for line in lines)
    # A limit tighter than the segmentation noise makes the held-out poses (and the fit) exceed it.
    assert register.main(arguments + ["--max-rms-offset-mm", "1e-9"]) == register.EXIT_FLAGGED
    assert all(line.endswith("EXCEEDS") for line in capsys.readouterr().out.splitlines() if "held out:" in line)
    document = json.loads((tmp_path / "registration.json").read_text(encoding="utf-8"))
    assert document["models"]["rigid"][register.PASS_TWO]["held_out"]["held_out_within_limits"] is False


def test_register_fits_a_manifest_pose_missing_from_the_plan_and_warns(analysis_holdout, tmp_path, capsys):
    truth, held_out = analysis_holdout["truth"], analysis_holdout["held_out"]
    pose_ids = [pose["pose_id"] for pose in truth["poses"]]
    recaptured = next(pose_id for pose_id in pose_ids if pose_id not in held_out)       # not in the plan
    never_captured = "planned_but_never_captured"                                        # not in the manifest
    plan = write_plan_csv(tmp_path / "poses.csv", truth, held_out, omit=(recaptured,), extra_rows=((never_captured, 1),))
    assert register.main(["--manifest", str(analysis_holdout["manifest"]), "--plan", str(plan),
                          "--out", str(tmp_path / "reg"), "--model", "rigid", "--no-figures"]) == register.EXIT_OK
    printed = capsys.readouterr().out
    assert f"WARNING: 1 of {len(pose_ids)} plan poses were not captured (1 of them tagged held out): {never_captured}" \
        in printed
    assert f"WARNING: 1 manifest pose(s) are not in the plan (re-captures?) and are treated as not held out: " \
           f"{recaptured}" in printed
    document = json.loads((tmp_path / "reg" / "registration.json").read_text(encoding="utf-8"))
    assert document["plan"]["poses_not_captured"] == [never_captured]
    assert document["plan"]["manifest_poses_not_in_plan"] == [recaptured]
    for pass_name in (register.PASS_ONE, register.PASS_TWO):
        block = document["models"]["rigid"][pass_name]
        by_id = {p["pose_id"]: p for p in block["poses"]}
        assert by_id[recaptured]["used"] and not by_id[recaptured]["held_out"]
        assert block["poses_used"] == len(pose_ids) - len(held_out)
        assert {p["pose_id"] for p in block["held_out"]["poses"]} == held_out


def test_register_rejects_a_plan_it_cannot_use(analysis_holdout, tmp_path, capsys):
    base = ["--manifest", str(analysis_holdout["manifest"]), "--out", str(tmp_path / "reg"), "--no-figures"]
    truth = analysis_holdout["truth"]
    no_column = write_plan_csv(tmp_path / "no_column.csv", truth, set(), holdout_column=False)
    assert register.main(base + ["--plan", str(no_column)]) == register.EXIT_INPUT_ERROR
    assert "no 'holdout' column" in capsys.readouterr().err
    bad_tag = tmp_path / "bad_tag.csv"
    lines = analysis_holdout["plan"].read_text(encoding="utf-8").splitlines()
    lines[1] = lines[1].rsplit(",", 1)[0] + ",yes"                    # the holdout cell of the first pose
    bad_tag.write_text("\n".join(lines), encoding="utf-8")
    assert register.main(base + ["--plan", str(bad_tag)]) == register.EXIT_INPUT_ERROR
    assert "expected 1 (held out) or 0" in capsys.readouterr().err
    assert register.main(base + ["--plan", str(tmp_path / "missing.csv")]) == register.EXIT_INPUT_ERROR
    assert "cannot read the plan" in capsys.readouterr().err


def test_report_and_compare_carry_the_held_out_poses(analysis_holdout, tmp_path):
    root, held_out = analysis_holdout["root"], analysis_holdout["held_out"]
    registration = root / "reg" / "registration.json"
    assert compare.main(["--registration-a", str(registration), "--registration-b", str(registration),
                         "--out", str(tmp_path / "cmp"), "--model", "rigid"]) == compare.EXIT_OK
    comparison = json.loads((tmp_path / "cmp" / "comparison.json").read_text(encoding="utf-8"))
    section = analysis_holdout["document"]["models"]["rigid"][register.PASS_TWO]["held_out"]
    assert comparison["models"]["rigid"]["held_out"]["count"] == {"a": len(held_out), "b": len(held_out)}
    assert comparison["models"]["rigid"]["held_out"]["rms_offset_residual_mm"]["a"] == section["rms_offset_residual_mm"]
    assert report.main(["--registration", str(registration), "--comparison", str(tmp_path / "cmp" / "comparison.json"),
                        "--out", str(tmp_path / "report.md")]) == report.EXIT_OK
    text = (tmp_path / "report.md").read_text(encoding="utf-8")
    assert text.count("### Held-out poses") == 2                       # one per model
    assert f"{len(held_out)} held-out poses were left out of the solve" in text
    marked = [line for line in text.splitlines() if line.endswith(f"| {report.HELD_OUT_MARK} |")
              and line.split(" | ")[0].lstrip("| ") in held_out]
    assert len(marked) == 2 * len(held_out)                              # the marked rows of the two per-pose tables
    assert "| held-out poses |" in text                                   # the comparison table
    # A registration without a plan has no such subsection.
    plain = tmp_path / "plain"
    register.main(["--manifest", str(analysis_holdout["manifest"]), "--out", str(plain), "--model", "rigid",
                   "--no-figures"])
    report.main(["--registration", str(plain / "registration.json"), "--out", str(tmp_path / "plain.md")])
    assert "Held-out poses" not in (tmp_path / "plain.md").read_text(encoding="utf-8")
    # Comparing a session with a plan against one without leaves the held-out figures out.
    compare.main(["--registration-a", str(registration), "--registration-b", str(plain / "registration.json"),
                  "--out", str(tmp_path / "cmp2"), "--model", "rigid"])
    assert "held_out" not in json.loads((tmp_path / "cmp2" / "comparison.json").read_text())["models"]["rigid"]


@pytest.mark.parametrize("tool", [simulate, register, residual_maps, compare, report])
def test_every_tool_answers_help(tool, capsys):
    with pytest.raises(SystemExit) as stop:
        tool.main(["--help"])
    assert stop.value.code == 0
    assert "usage" in capsys.readouterr().out
