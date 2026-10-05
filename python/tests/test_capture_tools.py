"""
Tests of the three capture tools (plan_poses, bootstrap, check_captures), run through their ``main(argv)``
functions on small synthetic sessions with a known sensor-to-base transform (code design, Section 9).

A session is built the way a real one is: plan_poses writes poses.csv, a board is rendered at every
planned pose with sphcal's synthetic renderer and written as .mc files named ``<pose_id>_Index<nn>.mc``,
and sphcal's make_manifest turns poses.csv and the capture folder into the manifest. The sessions use a
160 x 120 camera and few frames so that the whole file runs in well under a minute.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from planereg.capture import bootstrap, check_captures, plan_poses
from planereg.capture.common import EXIT_FLAGGED, EXIT_INPUT_ERROR, EXIT_OK
from sphcal.cli import make_manifest
from sphcal.cli.plan_poses import load_sensor_in_base
from sphcal.geometry.camera import PinholeCamera
from sphcal.geometry.transforms import RigidTransform
from sphcal.io.matcloud import write_matcloud
from sphcal.io.poses import CaptureRecord, TARGET_KIND_BOARD, load_manifest, write_manifest_json
from sphcal.simulate.synthetic import SyntheticSensorParameters, render_board_frame

# ---------------------------------------------------------------------------
# Test constants
# ---------------------------------------------------------------------------
TEST_CAMERA = PinholeCamera(160, 120, 172.04, 172.02, 73.08, 64.19)
"""The small camera of the rendered sessions (the one sphcal's own tests use)."""
TEST_BLOCK_PX = 2
"""Effective resolution block of the synthetic sensor at the small image size."""
TEST_SENSOR_TO_BASE = RigidTransform.from_rotation_vector_degrees([175.0, 4.0, -6.0], [350.0, -150.0, 900.0])
"""The true sensor-to-base transform of every synthetic session."""
ROUGH_ERROR = RigidTransform.from_rotation_vector_degrees([1.0, -1.0, 0.5], [8.0, -6.0, 5.0])
"""Error of the rough transform handed to the predicted-region segmentation (about 1.5 deg, 11 mm)."""
BOARD_HALF_SIZE_MM = (100.0, 75.0)
FRAMES_PER_POSE = 2
FRAME_FILE_FORMAT = "{pose_id}_Index{frame:02d}.mc"
HEADER_NAME = "synthetic"
HEADER_VERSION = 2

SESSION_PLAN_ARGUMENTS = ["--standoffs-mm", "700", "900", "--tilts-deg", "0", "20", "--azimuths-deg", "0", "90",
                          "--lateral-positions", "2", "2", "--lateral-fill", "0.3", "--edge-margin-px", "10"]
"""A plan of 24 poses on the small camera: two standoffs, two lateral rows and columns, tilts 0 and 20."""
SESSION_POSE_COUNT = 24

BOOTSTRAP_POSES = (  # (center in the sensor frame, tilt deg, azimuth deg)
    ((0.0, 0.0, 700.0), 0.0, 0.0),
    ((-90.0, -40.0, 600.0), 25.0, 0.0),
    ((90.0, -40.0, 800.0), 25.0, 120.0),
    ((90.0, 50.0, 650.0), 25.0, 240.0),
    ((-90.0, 50.0, 750.0), 20.0, 60.0),
    ((0.0, 10.0, 900.0), 30.0, 300.0),
)
"""Six hand-jogged-like board poses: spread through the field, at different depths and tilted in different
directions (the normal spread of the set is well above the registration minimum)."""

RECOVERY_ROTATION_TOLERANCE_DEG = 0.1
RECOVERY_TRANSLATION_TOLERANCE_MM = 1.0
"""Design Section 9: the bootstrap recovers the synthetic transform within 0.1 deg / 1 mm."""
SHIFT_MM = 10.0
"""Deliberate error added to one logged position."""
SHIFT_TOLERANCE_MM = 0.5
"""How closely the offset residual of the shifted pose must show the shift."""
SHIFTED_POSE_INDEX = 7
"""Index (in plan order) of the pose whose logged position is shifted."""
FLANGE_OFFSET_MM = 6.0
"""Distance D from the origin of the logged (flange-like) frame to the board's front face along its +z: the
logged frame sits this far BEHIND the board face, so the software must shift the logged plane by +D."""
FLANGE_RECOVERY_TRANSLATION_TOLERANCE_MM = 1.0
"""With the right --target-offset-mm the recovered sensor-to-base translation is this close to the truth."""
FLANGE_UNCORRECTED_MIN_TRANSLATION_ERROR_MM = 0.5 * FLANGE_OFFSET_MM
"""Without --target-offset-mm the registration absorbs most of the constant offset D into the translation of the
transform, so the transform is wrong by roughly D (at least half of it) while the residuals stay small."""
PLAN_COUNT_RANGE = (150, 250)
"""'About 200 poses' for the default plan: the accepted range."""
ROTATION_ATOL = 1.0e-9
POSITION_ATOL_MM = 1.0e-6
WALL_OFFSET_MM = 300.0
"""Distance of the background wall behind the board in the clutter session."""
WALL_HALF_SIZE_MM = (3000.0, 3000.0)
"""A wall large enough to fill the whole field."""


# ---------------------------------------------------------------------------
# Synthetic session builders
# ---------------------------------------------------------------------------
def sensor_parameters(camera: PinholeCamera = TEST_CAMERA) -> SyntheticSensorParameters:
    """The indicative sensor model on the given (small) camera."""
    base = SyntheticSensorParameters.vsx3000_indicative()
    return SyntheticSensorParameters(**{**base.__dict__, "camera": camera, "effective_block_px": TEST_BLOCK_PX})


def zero_error_field(u, v, rho, s_u, s_v, curvature):
    """No injected range error: the registration should then recover the transform from noise alone."""
    return np.zeros_like(rho)


def write_json_file(path: Path, document: dict) -> Path:
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def sensor_in_base_file(path: Path, transform: RigidTransform) -> Path:
    return write_json_file(path, {"matrix": transform.as_matrix().reshape(-1).tolist()})


def write_camera_file(path: Path, camera: PinholeCamera) -> Path:
    """An .mc file that only carries the camera (header and image size), for --camera."""
    header = {"fx": camera.focal_x_px, "fy": camera.focal_y_px, "cx": camera.principal_x_px,
              "cy": camera.principal_y_px, "h": camera.width, "v": camera.height,
              "cameraName": HEADER_NAME, "version": HEADER_VERSION}
    write_matcloud(path, header, {"XYZ": np.zeros((camera.height, camera.width, 3), dtype=np.float32)})
    return path


def render_frame(params: SyntheticSensorParameters, board_to_sensor: RigidTransform, rng: np.random.Generator,
                 wall: bool) -> np.ndarray:
    """One XYZ frame of the board; with ``wall`` a plane WALL_OFFSET_MM behind it fills the rest of the field
    (the nearer surface wins where both are hit)."""
    board = render_board_frame(params, board_to_sensor, BOARD_HALF_SIZE_MM, zero_error_field, rng).xyz
    if not wall:
        return board
    wall_pose = RigidTransform(board_to_sensor.rotation,
                               board_to_sensor.translation - WALL_OFFSET_MM * board_to_sensor.rotation[:, 2])
    background = render_board_frame(params, wall_pose, WALL_HALF_SIZE_MM, zero_error_field, rng).xyz
    use_board = (board[..., 2] > 0.0) & ((background[..., 2] <= 0.0) | (board[..., 2] < background[..., 2]))
    return np.where(use_board[..., None], board, background)


def write_captures(directory: Path, poses: dict[str, RigidTransform], true_sensor_to_base: RigidTransform,
                   camera: PinholeCamera = TEST_CAMERA, wall: bool = False, seed: int = 3) -> None:
    """Render FRAMES_PER_POSE frames of the board at every pose (T -> B, keyed by pose id) and write them as
    ``<pose_id>_Index<nn>.mc`` into ``directory``."""
    directory.mkdir(parents=True, exist_ok=True)
    params, rng = sensor_parameters(camera), np.random.default_rng(seed)
    base_to_sensor = true_sensor_to_base.inverse()
    header = {"fx": camera.focal_x_px, "fy": camera.focal_y_px, "cx": camera.principal_x_px,
              "cy": camera.principal_y_px, "h": camera.width, "v": camera.height,
              "cameraName": HEADER_NAME, "version": HEADER_VERSION}
    for pose_id, board_to_base in poses.items():
        for frame in range(FRAMES_PER_POSE):
            xyz = render_frame(params, base_to_sensor.compose(board_to_base), rng, wall)
            write_matcloud(directory / FRAME_FILE_FORMAT.format(pose_id=pose_id, frame=frame), header, {"XYZ": xyz})


def read_plan_rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def row_transform(row: dict, prefix: str = "") -> RigidTransform:
    """The pose of a poses.csv row (target pose, or with prefix ``approach_`` the approach pose), T -> B."""
    if prefix:
        position = [float(row[f"{prefix}{axis}_mm"]) for axis in "xyz"]
    else:
        position = [float(row[f"base_{axis}_mm"]) for axis in "xyz"]
    rotation = np.array([float(row[f"{prefix}r{i}{j}"]) for i in range(3) for j in range(3)]).reshape(3, 3)
    return RigidTransform(rotation, position)


def build_session(root: Path, plan_arguments: list[str], wall: bool = False) -> dict:
    """Plan, render and make the manifest of one session; returns its paths and the planned poses."""
    root.mkdir(parents=True, exist_ok=True)
    sensor_file = sensor_in_base_file(root / "true_sensor_in_base.json", TEST_SENSOR_TO_BASE)
    camera_file = write_camera_file(root / "camera.mc", TEST_CAMERA)
    plan_dir = root / "plan"
    assert plan_poses.main(["--sensor-in-base", str(sensor_file), "--camera", str(camera_file), "--out", str(plan_dir),
                            *plan_arguments]) == EXIT_OK
    rows = read_plan_rows(plan_dir / plan_poses.PLAN_CSV_NAME)
    poses = {row["pose_id"]: row_transform(row) for row in rows}
    write_captures(root / "captures", poses, TEST_SENSOR_TO_BASE, wall=wall)
    manifest = root / "manifest.json"
    assert make_manifest.main(["--pose-log", str(plan_dir / plan_poses.PLAN_CSV_NAME), "--captures",
                               str(root / "captures"), "--format", "json", "--out", str(manifest)]) == EXIT_OK
    return {"root": root, "manifest": manifest, "plan_dir": plan_dir, "rows": rows, "poses": poses}


@pytest.fixture(scope="module")
def session(tmp_path_factory) -> dict:
    """The clean 24-pose session."""
    return build_session(tmp_path_factory.mktemp("session"), SESSION_PLAN_ARGUMENTS)


@pytest.fixture(scope="module")
def rough_file(tmp_path_factory) -> Path:
    """A rough sensor-to-base file: the true transform with ROUGH_ERROR applied."""
    return sensor_in_base_file(tmp_path_factory.mktemp("rough") / "rough.json", TEST_SENSOR_TO_BASE.compose(ROUGH_ERROR))


def bootstrap_session(root: Path, poses_spec=BOOTSTRAP_POSES, wall: bool = False, parallel: bool = False) -> Path:
    """Render a few hand-jogged-like poses and write a manifest (as make_manifest would) for them. With
    ``parallel`` every board gets the orientation of the first one (all normals equal, so the set is degenerate)."""
    poses = {}
    first_rotation = None
    for index, (center, tilt, azimuth) in enumerate(poses_spec):
        center = np.array(center)
        rotation = plan_poses.board_rotation_in_sensor(center, tilt, azimuth)
        first_rotation = rotation if first_rotation is None else first_rotation
        board_to_sensor = RigidTransform(first_rotation if parallel else rotation, center)
        poses[f"jog{index}_x"] = TEST_SENSOR_TO_BASE.compose(board_to_sensor)
    write_captures(root / "captures", poses, TEST_SENSOR_TO_BASE, wall=wall)
    records = [CaptureRecord(
        path=root / "captures" / FRAME_FILE_FORMAT.format(pose_id=pose_id, frame=frame), pose_id=pose_id,
        frame_index=frame, target_kind=TARGET_KIND_BOARD, sphere_radius_mm=None, board_half_size_mm=BOARD_HALF_SIZE_MM,
        target_pose_positioner=pose) for pose_id, pose in poses.items() for frame in range(FRAMES_PER_POSE)]
    return write_manifest_json(root / "manifest.json", records)


def shifted_manifest(manifest: Path, pose_id: str, out: Path) -> Path:
    """A copy of the manifest in which every record of ``pose_id`` has its logged position moved by SHIFT_MM
    along the board's own normal (what a wrong tool frame or a mis-logged position looks like)."""
    records = load_manifest(manifest)
    for record in records:
        if record.pose_id == pose_id:
            pose = record.target_pose_positioner
            record.target_pose_positioner = RigidTransform(pose.rotation,
                                                           pose.translation + SHIFT_MM * pose.rotation[:, 2])
    return write_manifest_json(out, records)


def flange_logged_manifest(manifest: Path, out: Path, offset_mm: float = FLANGE_OFFSET_MM) -> Path:
    """A copy of the manifest in which every logged pose is a flange-like frame ``offset_mm`` BEHIND the board face:
    the logged origin is moved by -offset_mm along the board's own z axis (the rotation is unchanged). This is
    what a technician logs when the controller reports the flange pose and the board face is ``offset_mm`` ahead
    of the flange along its +z."""
    records = load_manifest(manifest)
    for record in records:
        pose = record.target_pose_positioner
        record.target_pose_positioner = RigidTransform(pose.rotation, pose.translation - offset_mm * pose.rotation[:, 2])
    return write_manifest_json(out, records)


def rotation_error_deg(a: RigidTransform, b: RigidTransform) -> float:
    return a.difference_from(b)[1]


# ---------------------------------------------------------------------------
# plan_poses
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def default_plan(tmp_path_factory) -> dict:
    """The default plan on the 640 x 480 indicative camera (geometry only, nothing is rendered)."""
    root = tmp_path_factory.mktemp("default_plan")
    camera = SyntheticSensorParameters.vsx3000_indicative().camera
    sensor_file = sensor_in_base_file(root / "sensor_in_base.json", TEST_SENSOR_TO_BASE)
    camera_file = write_camera_file(root / "camera.mc", camera)
    assert plan_poses.main(["--sensor-in-base", str(sensor_file), "--camera", str(camera_file),
                            "--out", str(root / "plan")]) == EXIT_OK
    return {"dir": root / "plan", "camera": camera, "rows": read_plan_rows(root / "plan" / plan_poses.PLAN_CSV_NAME)}


def test_default_plan_has_about_two_hundred_poses(default_plan):
    assert PLAN_COUNT_RANGE[0] <= len(default_plan["rows"]) <= PLAN_COUNT_RANGE[1]
    for name in (plan_poses.PLAN_CSV_NAME, plan_poses.PLAN_SUMMARY_NAME, plan_poses.PLAN_FIGURE_NAME):
        assert (default_plan["dir"] / name).stat().st_size > 0


def test_every_planned_board_is_inside_the_image(default_plan):
    camera, margin = default_plan["camera"], plan_poses.PlanParameters().edge_margin_px
    half_size = plan_poses.PlanParameters().board_half_size_mm
    base_to_sensor = TEST_SENSOR_TO_BASE.inverse()
    for row in default_plan["rows"]:
        board_to_sensor = base_to_sensor.compose(row_transform(row))
        corners = [board_to_sensor.apply_points([sx * half_size[0], sy * half_size[1], 0.0])
                   for sx in (-1, 1) for sy in (-1, 1)]
        u, v, in_front = camera.project(np.array(corners))
        assert in_front.all(), row["pose_id"]
        assert np.all((u >= margin) & (u <= camera.width - 1 - margin)
                      & (v >= margin) & (v <= camera.height - 1 - margin)), row["pose_id"]
        # Plan coordinates: the standoff is the sensor z of the board center, and the board faces the sensor.
        assert np.isclose(board_to_sensor.translation[2], float(row["standoff_mm"]), atol=POSITION_ATOL_MM)
        assert board_to_sensor.rotation[:, 2] @ board_to_sensor.translation < 0.0


def test_planned_tilt_matches_the_tilt_column(default_plan):
    """The angle between the board normal and the line of sight is the tilt (the board faces the sensor at tilt 0)."""
    base_to_sensor = TEST_SENSOR_TO_BASE.inverse()
    for row in default_plan["rows"]:
        board_to_sensor = base_to_sensor.compose(row_transform(row))
        to_sensor = -board_to_sensor.translation / np.linalg.norm(board_to_sensor.translation)
        angle = np.degrees(np.arccos(np.clip(board_to_sensor.rotation[:, 2] @ to_sensor, -1.0, 1.0)))
        assert np.isclose(angle, float(row["tilt_deg"]), atol=1.0e-6), row["pose_id"]


def test_approach_columns_are_the_stated_retreat_and_rotation_from_the_target(default_plan):
    defaults = plan_poses.PlanParameters()
    for row in default_plan["rows"]:
        assert all(f"approach_{name}" in row for name in ("x_mm", "y_mm", "z_mm", "r00", "r22"))
        target, approach = row_transform(row), row_transform(row, "approach_")
        relative = target.inverse().compose(approach)       # the approach pose seen from the target tool frame
        assert np.allclose(relative.translation, [0.0, 0.0, -defaults.approach_retreat_mm], atol=POSITION_ATOL_MM)
        assert np.allclose(relative.rotation,
                           Rotation.from_rotvec(np.radians(defaults.approach_rotation_deg) * np.array([1.0, 0.0, 0.0])
                                                ).as_matrix(), atol=ROTATION_ATOL)


def test_approach_pose_composition_order():
    """A = T o D with D a displacement in the tool frame: the retreat is along the board's own -z AFTER the
    rotation about its own x axis, so the origin moves by -retreat * (target z axis) exactly."""
    target = RigidTransform.from_rotation_vector_degrees([20.0, -35.0, 50.0], [100.0, 200.0, 300.0])
    approach = plan_poses.approach_pose_in_base(target, retreat_mm=40.0, rotation_deg=3.0)
    assert np.allclose(approach.translation, target.translation - 40.0 * target.rotation[:, 2])
    assert np.allclose(approach.rotation[:, 0], target.rotation[:, 0])         # the x axis is the rotation axis
    assert np.isclose(np.degrees(np.arccos(approach.rotation[:, 2] @ target.rotation[:, 2])), 3.0)


def test_poses_csv_is_accepted_by_make_manifest_as_a_pose_log(default_plan):
    messages = make_manifest.Messages()
    logged = make_manifest.read_pose_log(default_plan["dir"] / plan_poses.PLAN_CSV_NAME, messages)
    assert not messages.errors and not messages.warnings
    assert len(logged) == len(default_plan["rows"])
    for pose, row in zip(logged, default_plan["rows"]):
        assert pose.pose_id == row["pose_id"] and pose.kind == TARGET_KIND_BOARD
        assert pose.half_size_mm == plan_poses.PlanParameters().board_half_size_mm
        target = row_transform(row)
        assert np.allclose(pose.pose.translation, target.translation) and np.allclose(pose.pose.rotation, target.rotation)


def test_poses_csv_has_sphcal_columns_then_the_plan_columns(default_plan):
    with (default_plan["dir"] / plan_poses.PLAN_CSV_NAME).open(newline="") as handle:
        columns = next(csv.reader(handle))
    from sphcal.cli.plan_poses import POSE_CSV_COLUMNS
    assert columns == list(POSE_CSV_COLUMNS) + list(plan_poses.PLAN_EXTRA_COLUMNS)
    assert columns[-len(plan_poses.PLAN_EXTRA_COLUMNS):][:6] == ["standoff_mm", "tilt_deg", "azimuth_deg", "approach_x_mm",
                                                                 "approach_y_mm", "approach_z_mm"]
    assert columns[-1] == "approach_rotvec_z_deg"
    # The approach rotation follows its matrix columns, in the order matrix, quaternion, rotation vector.
    assert columns[columns.index("approach_r22") + 1:] == [
        "approach_quat_w", "approach_quat_x", "approach_quat_y", "approach_quat_z",
        "approach_rotvec_x_deg", "approach_rotvec_y_deg", "approach_rotvec_z_deg"]


def test_approach_quaternion_and_rotation_vector_agree_with_the_approach_matrix(default_plan):
    """The approach pose's quaternion (w >= 0) and rotation vector (degrees) describe the approach rotation matrix,
    (SciPy's conventions, as sphcal's pose_row uses for the target pose's quat_* and rotvec_* columns)."""
    for row in default_plan["rows"]:
        matrix = row_transform(row, "approach_").rotation
        quaternion = np.array([float(row[f"approach_quat_{axis}"]) for axis in "wxyz"])
        rotation_vector_deg = np.array([float(row[f"approach_rotvec_{axis}_deg"]) for axis in "xyz"])
        assert quaternion[0] >= 0.0 and np.isclose(np.linalg.norm(quaternion), 1.0, atol=ROTATION_ATOL)
        # SciPy takes the scalar part last.
        assert np.allclose(Rotation.from_quat(np.roll(quaternion, -1)).as_matrix(), matrix, atol=ROTATION_ATOL)
        assert np.allclose(Rotation.from_rotvec(np.radians(rotation_vector_deg)).as_matrix(), matrix, atol=ROTATION_ATOL)


def test_plan_summary_reports_spreads_errors_and_capture_totals(default_plan):
    text = (default_plan["dir"] / plan_poses.PLAN_SUMMARY_NAME).read_text()
    count = len(default_plan["rows"])
    for expected in ("normal spread", "similarity spread", "predicted rotation RMS error",
                     "predicted translation RMS error", "dropped", f"Total: {count} poses",
                     f"2 x {count} poses = {2 * count} pose visits", f"{2 * count * 5} capture files"):
        assert expected in text, expected


def test_plan_quality_uses_base_normals_and_sensor_offsets(default_plan):
    """The summary's spreads are those of the planned base-frame normals and of [m, -d / max d]."""
    from planereg.core.registration import spread
    rows, base_to_sensor = default_plan["rows"], TEST_SENSOR_TO_BASE.inverse()
    normals = np.array([row_transform(row).rotation[:, 2] for row in rows])
    offsets = np.array([-(base_to_sensor.compose(row_transform(row)).rotation[:, 2]
                          @ base_to_sensor.compose(row_transform(row)).translation) for row in rows])
    expected = (spread(normals), spread(np.column_stack([normals, -offsets / offsets.max()])))
    text = (default_plan["dir"] / plan_poses.PLAN_SUMMARY_NAME).read_text()
    assert f"normal spread      {expected[0]:.4f}" in text
    assert f"similarity spread  {expected[1]:.4f}" in text


def test_target_pose_count_scales_the_grid(tmp_path):
    camera = SyntheticSensorParameters.vsx3000_indicative().camera
    counts = {}
    for target in (60, 400):
        params = plan_poses.PlanParameters(target_pose_count=target)
        grid = plan_poses.choose_lateral_grid(params, camera)
        counts[target] = len(plan_poses.plan_in_sensor_frame(params, camera, grid)[0])
        assert abs(counts[target] - target) <= 0.25 * target
    assert counts[60] < counts[400]


def test_holdout_fraction_and_seed_are_reproducible(default_plan):
    held = [row["pose_id"] for row in default_plan["rows"] if row["holdout"] == "1"]
    assert len(held) == round(plan_poses.PlanParameters().holdout_fraction * len(default_plan["rows"]))
    poses_a, _ = plan_poses.plan_in_sensor_frame(plan_poses.PlanParameters(), default_plan["camera"], (3, 3))
    poses_b = [pose for pose in poses_a]
    plan_poses.tag_holdout(poses_a, 0.2, 0)
    plan_poses.tag_holdout(poses_b, 0.2, 0)
    assert [p.holdout for p in poses_a] == [p.holdout for p in poses_b]


def test_plan_accepts_a_bootstrap_json_and_rejects_bad_input(tmp_path, capsys):
    camera_file = write_camera_file(tmp_path / "camera.mc", TEST_CAMERA)
    out = tmp_path / "plan"
    base = ["--camera", str(camera_file), "--out", str(out), *SESSION_PLAN_ARGUMENTS]
    # A bootstrap JSON that was not Success is used but warned about.
    failed = write_json_file(tmp_path / "failed.json", {"matrix": TEST_SENSOR_TO_BASE.as_matrix().reshape(-1).tolist(),
                                                        "status": "DegeneratePoses"})
    assert plan_poses.main(["--sensor-in-base", str(failed), *base]) == EXIT_OK
    assert "WARNING:" in capsys.readouterr().out
    # Input errors give exit code 2 and an ERROR: message that says what to fix.
    assert plan_poses.main(["--sensor-in-base", str(tmp_path / "missing.json"), *base]) == EXIT_INPUT_ERROR
    assert capsys.readouterr().err.startswith("ERROR:")
    good = sensor_in_base_file(tmp_path / "good.json", TEST_SENSOR_TO_BASE)
    assert plan_poses.main(["--sensor-in-base", str(good), "--camera", str(camera_file), "--out", str(out),
                            "--tilts-deg", "95"]) == EXIT_INPUT_ERROR
    assert "ERROR:" in capsys.readouterr().err
    assert plan_poses.main(["--sensor-in-base", str(good), "--camera", str(camera_file), "--out", str(out),
                            "--standoffs-mm", "100"]) == EXIT_INPUT_ERROR      # every board falls outside the image
    assert "ERROR:" in capsys.readouterr().err


# ---------------------------------------------------------------------------
# bootstrap
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def bootstrap_run(tmp_path_factory) -> dict:
    root = tmp_path_factory.mktemp("bootstrap")
    manifest = bootstrap_session(root)
    out = root / "sensor_in_base.json"
    code = bootstrap.main(["--manifest", str(manifest), "--out", str(out)])
    return {"root": root, "manifest": manifest, "out": out, "code": code, "document": json.loads(out.read_text())}


def test_bootstrap_recovers_the_transform_from_six_poses(bootstrap_run):
    assert bootstrap_run["code"] == EXIT_OK
    document = bootstrap_run["document"]
    assert document["status"] == "Success" and len(document["poses"]) == len(BOOTSTRAP_POSES)
    solved = RigidTransform.from_matrix(np.array(document["matrix"]).reshape(4, 4))
    translation_error, rotation_error = solved.difference_from(TEST_SENSOR_TO_BASE)
    assert rotation_error < RECOVERY_ROTATION_TOLERANCE_DEG, rotation_error
    assert translation_error < RECOVERY_TRANSLATION_TOLERANCE_MM, translation_error
    assert document["normal_spread"] > bootstrap.BootstrapParameters().minimum_normal_spread
    assert all(pose["used"] and pose["method"] == "closest_large_plane" for pose in document["poses"])


def test_bootstrap_json_is_read_by_sphcal_and_by_plan_poses(bootstrap_run, tmp_path):
    transform, _, _ = load_sensor_in_base(bootstrap_run["out"], residual_warn_mm=1.0)
    assert transform.difference_from(TEST_SENSOR_TO_BASE)[0] < RECOVERY_TRANSLATION_TOLERANCE_MM
    camera_file = write_camera_file(tmp_path / "camera.mc", TEST_CAMERA)
    assert plan_poses.main(["--sensor-in-base", str(bootstrap_run["out"]), "--camera", str(camera_file),
                            "--out", str(tmp_path / "plan"), *SESSION_PLAN_ARGUMENTS]) == EXIT_OK
    rows = read_plan_rows(tmp_path / "plan" / plan_poses.PLAN_CSV_NAME)
    assert len(rows) == SESSION_POSE_COUNT


def test_bootstrap_prints_table_and_transform(bootstrap_run, capsys, tmp_path):
    assert bootstrap.main(["--manifest", str(bootstrap_run["manifest"]), "--out", str(tmp_path / "again.json")]) == EXIT_OK
    printed = capsys.readouterr().out
    for expected in ("pose_id", "method", "pixels", "plane_rms_mm", "normal_deg", "offset_mm",
                     "Sensor-to-base transform", "Registration status: Success"):
        assert expected in printed, expected


def test_bootstrap_exits_one_without_tilt_variation(tmp_path, capsys):
    """Six parallel boards have the same normal: the normal spread gate refuses, with advice to tilt."""
    manifest = bootstrap_session(tmp_path, parallel=True)
    out = tmp_path / "sensor_in_base.json"
    assert bootstrap.main(["--manifest", str(manifest), "--out", str(out)]) == EXIT_FLAGGED
    assert "tilt the board more" in capsys.readouterr().err
    document = json.loads(out.read_text())
    assert document["status"] == "DegeneratePoses" and document["matrix"] is None


def test_bootstrap_exits_one_with_too_few_poses_and_two_on_bad_manifest(tmp_path, capsys):
    manifest = bootstrap_session(tmp_path, BOOTSTRAP_POSES[:3])
    assert bootstrap.main(["--manifest", str(manifest), "--out", str(tmp_path / "out.json")]) == EXIT_FLAGGED
    err = capsys.readouterr().err
    assert "ERROR:" in err and "capture more board poses" in err
    assert bootstrap.main(["--manifest", str(tmp_path / "missing.json"), "--out", str(tmp_path / "out.json")]) == EXIT_INPUT_ERROR
    assert "ERROR:" in capsys.readouterr().err


def test_bootstrap_target_offset_option_is_parsed_into_the_parameters():
    default = bootstrap.parameters_from_arguments(bootstrap.build_parser().parse_args(
        ["--manifest", "m.json", "--out", "o.json"]))
    given = bootstrap.parameters_from_arguments(bootstrap.build_parser().parse_args(
        ["--manifest", "m.json", "--out", "o.json", "--target-offset-mm", str(FLANGE_OFFSET_MM)]))
    assert default.target_offset_mm == bootstrap.BootstrapParameters().target_offset_mm == 0.0
    assert given.target_offset_mm == FLANGE_OFFSET_MM


def test_bootstrap_target_offset_corrects_a_logged_flange_pose(tmp_path):
    """The logged poses are D behind the board face: --target-offset-mm D recovers the transform, and without it
    the transform is off by about D (the constant offset is absorbed into the translation)."""
    manifest = flange_logged_manifest(bootstrap_session(tmp_path), tmp_path / "flange_manifest.json")
    corrected_out, plain_out = tmp_path / "corrected.json", tmp_path / "plain.json"
    assert bootstrap.main(["--manifest", str(manifest), "--out", str(corrected_out),
                           "--target-offset-mm", str(FLANGE_OFFSET_MM)]) == EXIT_OK
    corrected = json.loads(corrected_out.read_text())
    solved = RigidTransform.from_matrix(np.array(corrected["matrix"]).reshape(4, 4))
    translation_error, rotation_error = solved.difference_from(TEST_SENSOR_TO_BASE)
    assert rotation_error < RECOVERY_ROTATION_TOLERANCE_DEG and translation_error < FLANGE_RECOVERY_TRANSLATION_TOLERANCE_MM
    assert corrected["rms_offset_residual_mm"] < FLANGE_RECOVERY_TRANSLATION_TOLERANCE_MM
    # The same manifest without the option: the transform is wrong by about D.
    bootstrap.main(["--manifest", str(manifest), "--out", str(plain_out)])
    plain = json.loads(plain_out.read_text())
    plain_solved = RigidTransform.from_matrix(np.array(plain["matrix"]).reshape(4, 4))
    assert plain_solved.difference_from(TEST_SENSOR_TO_BASE)[0] > FLANGE_UNCORRECTED_MIN_TRANSLATION_ERROR_MM


# ---------------------------------------------------------------------------
# check_captures
# ---------------------------------------------------------------------------
def test_check_passes_a_clean_session(session, rough_file, tmp_path, capsys):
    report = tmp_path / "check.json"
    code = check_captures.main(["--manifest", str(session["manifest"]), "--sensor-in-base", str(rough_file),
                                "--out", str(report)])
    printed = capsys.readouterr()
    assert code == EXIT_OK, printed.out + printed.err
    assert f"VERDICT: all {SESSION_POSE_COUNT} poses passed." in printed.out
    document = json.loads(report.read_text())
    assert document["n_flagged"] == 0 and document["predicted_region_used"]
    assert all(pose["method"] == "predicted_region" for pose in document["poses"])
    assert document["registration"]["status"] in ("Success", "ResidualsExceedThreshold")
    assert document["normal_spread"] > 0.05 and document["similarity_spread"] > 0.01
    solved = RigidTransform.from_matrix(np.array(document["registration"]["sensor_to_base"]).reshape(4, 4))
    assert solved.difference_from(TEST_SENSOR_TO_BASE)[0] < RECOVERY_TRANSLATION_TOLERANCE_MM


def test_check_passes_a_clean_session_without_a_rough_transform(session, capsys):
    assert check_captures.main(["--manifest", str(session["manifest"])]) == EXIT_OK
    assert "closest_large_plane" in capsys.readouterr().out


def test_check_flags_a_pose_whose_logged_position_was_shifted(session, rough_file, tmp_path, capsys):
    pose_id = list(session["poses"])[SHIFTED_POSE_INDEX]
    manifest = shifted_manifest(session["manifest"], pose_id, tmp_path / "shifted.json")
    report = tmp_path / "check.json"
    code = check_captures.main(["--manifest", str(manifest), "--sensor-in-base", str(rough_file), "--out", str(report)])
    printed = capsys.readouterr()
    assert code == EXIT_FLAGGED
    assert f"WARNING: pose {pose_id}:" in printed.err and "offset residual too large" in printed.err
    document = json.loads(report.read_text())
    by_id = {pose["pose_id"]: pose for pose in document["poses"]}
    assert any(check_captures.FLAG_OFFSET in flag for flag in by_id[pose_id]["flags"])
    # The registration leaves the wrong pose out, so its residual shows the shift and no innocent pose is flagged.
    assert [pose["pose_id"] for pose in document["poses"] if pose["flags"]] == [pose_id]
    assert not by_id[pose_id]["in_fit"] and abs(abs(by_id[pose_id]["offset_residual_mm"]) - SHIFT_MM) < SHIFT_TOLERANCE_MM
    assert "VERDICT: 1 of" in printed.out
    # Without the outlier rounds the wrong pose pulls the solution and the verdict is still flagged.
    assert check_captures.main(["--manifest", str(manifest), "--sensor-in-base", str(rough_file),
                                "--outlier-rounds", "0"]) == EXIT_FLAGGED


def test_check_target_offset_option_is_parsed_into_the_parameters():
    default = check_captures.parameters_from_arguments(check_captures.build_parser().parse_args(["--manifest", "m.json"]))
    given = check_captures.parameters_from_arguments(check_captures.build_parser().parse_args(
        ["--manifest", "m.json", "--target-offset-mm", str(FLANGE_OFFSET_MM)]))
    assert default.target_offset_mm == check_captures.CheckParameters().target_offset_mm == 0.0
    assert given.target_offset_mm == FLANGE_OFFSET_MM


def test_check_target_offset_corrects_a_logged_flange_pose(session, tmp_path, capsys):
    """The logged poses are D behind the board face: with --target-offset-mm D the set passes and the recovered
    transform is the true one; without it the recovered transform is off by about D."""
    manifest = flange_logged_manifest(session["manifest"], tmp_path / "flange_manifest.json")
    corrected_report, plain_report = tmp_path / "corrected.json", tmp_path / "plain.json"
    code = check_captures.main(["--manifest", str(manifest), "--target-offset-mm", str(FLANGE_OFFSET_MM),
                                "--out", str(corrected_report)])
    printed = capsys.readouterr()
    assert code == EXIT_OK, printed.out + printed.err
    corrected = json.loads(corrected_report.read_text())
    assert corrected["parameters"]["target_offset_mm"] == FLANGE_OFFSET_MM and corrected["n_flagged"] == 0
    solved = RigidTransform.from_matrix(np.array(corrected["registration"]["sensor_to_base"]).reshape(4, 4))
    assert solved.difference_from(TEST_SENSOR_TO_BASE)[0] < FLANGE_RECOVERY_TRANSLATION_TOLERANCE_MM
    # The same manifest without the option: the transform is wrong by about D.
    check_captures.main(["--manifest", str(manifest), "--out", str(plain_report)])
    plain = json.loads(plain_report.read_text())
    plain_solved = RigidTransform.from_matrix(np.array(plain["registration"]["sensor_to_base"]).reshape(4, 4))
    assert plain_solved.difference_from(TEST_SENSOR_TO_BASE)[0] > FLANGE_UNCORRECTED_MIN_TRANSLATION_ERROR_MM


def test_check_border_test_uses_the_mask_not_the_background(tmp_path, capsys):
    """With a wall behind the board the valid pixels fill the whole image, so a test on valid pixels would flag
    every pose; the mask of the board stays clear of the border and nothing is flagged."""
    session = build_session(tmp_path / "wall", SESSION_PLAN_ARGUMENTS, wall=True)
    rough = sensor_in_base_file(tmp_path / "rough.json", TEST_SENSOR_TO_BASE.compose(ROUGH_ERROR))
    code = check_captures.main(["--manifest", str(session["manifest"]), "--sensor-in-base", str(rough)])
    printed = capsys.readouterr()
    assert code == EXIT_OK, printed.out + printed.err
    assert "touches the image border" not in printed.err + printed.out
    # And the same wall session is also found without a prediction (the closest large plane is the board).
    assert check_captures.main(["--manifest", str(session["manifest"])]) == EXIT_OK


def test_check_flags_a_board_cut_by_the_image_border(tmp_path, capsys):
    cut = ((0.0, 0.0, 700.0), 0.0, 0.0), ((-90.0, -40.0, 600.0), 25.0, 0.0), ((90.0, -40.0, 800.0), 25.0, 120.0), \
          ((90.0, 50.0, 650.0), 25.0, 240.0), ((-90.0, 50.0, 750.0), 20.0, 60.0), ((-290.0, 10.0, 700.0), 10.0, 300.0)
    manifest = bootstrap_session(tmp_path, cut)
    code = check_captures.main(["--manifest", str(manifest)])
    printed = capsys.readouterr()
    assert code == EXIT_FLAGGED
    assert "WARNING: pose jog5_x:" in printed.err and "image border" in printed.err
    assert "pose jog0_x:" not in printed.err


def test_check_reports_an_unreadable_manifest_and_missing_files(session, tmp_path, capsys):
    assert check_captures.main(["--manifest", str(tmp_path / "missing.json")]) == EXIT_INPUT_ERROR
    assert capsys.readouterr().err.startswith("ERROR:")
    assert check_captures.main(["--manifest", str(session["manifest"]), "--pose-log", str(tmp_path / "no.csv")]) == EXIT_INPUT_ERROR
    assert "no.csv" in capsys.readouterr().err
    assert check_captures.main(["--manifest", str(session["manifest"]), "--sensor-in-base",
                                str(tmp_path / "no.json")]) == EXIT_INPUT_ERROR
    assert "no.json" in capsys.readouterr().err
    # A capture file that cannot be read flags its pose instead of stopping the run.
    records = load_manifest(session["manifest"])
    victim = records[0].pose_id
    for record in records:
        if record.pose_id == victim:
            record.path = tmp_path / "gone.mc"
    manifest = write_manifest_json(tmp_path / "broken.json", records)
    assert check_captures.main(["--manifest", str(manifest)]) == EXIT_FLAGGED
    assert f"WARNING: pose {victim}: capture files could not be read" in capsys.readouterr().err


# ---------------------------------------------------------------------------
# Joint-sign report
# ---------------------------------------------------------------------------
JOINT_COLUMNS = [f"j{i}" for i in range(1, 7)] + [f"approach_j{i}" for i in range(1, 7)]


def write_pose_log(path: Path, pose_ids: list[str], motions: np.ndarray, extra_columns: bool = True) -> Path:
    """A pose log whose final motions (target minus approach) of joint i at pose k are motions[k, i]."""
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["pose_id"] + (JOINT_COLUMNS if extra_columns else []))
        for pose_id, motion in zip(pose_ids, motions):
            approach = np.full(6, 30.0)
            writer.writerow([pose_id] + (list(approach + motion) + list(approach) if extra_columns else []))
    return path


def test_joint_sign_fractions_are_the_majority_sign_fraction(tmp_path):
    pose_ids = [f"p{i}" for i in range(10)]
    motions = np.ones((10, 6))
    motions[:, 1] = -1.0                      # joint 2: always negative
    motions[:3, 2] = -1.0                     # joint 3: 7 positive, 3 negative
    motions[::2, 3] = -1.0                    # joint 4: 5 / 5
    motions[:, 4] = 0.0                       # joint 5: never moves
    log = write_pose_log(tmp_path / "pose_log.csv", pose_ids, motions)
    read, note = check_captures.read_joint_motions(log, set(pose_ids))
    assert note is None
    rows = {row.joint: row for row in check_captures.joint_sign_rows(read)}
    assert rows[1].majority_fraction == 1.0 and rows[1].positive == 10
    assert rows[2].majority_fraction == 1.0 and rows[2].negative == 10
    assert rows[3].majority_fraction == 0.7
    assert rows[4].majority_fraction == 0.5
    assert np.isnan(rows[5].majority_fraction) and rows[5].zero == 10
    # Only the poses of the manifest count.
    read_subset, _ = check_captures.read_joint_motions(log, set(pose_ids[:3]))
    assert len(read_subset[3]) == 3


def test_joint_sign_report_is_skipped_with_a_note_when_columns_are_absent(tmp_path):
    log = write_pose_log(tmp_path / "pose_log.csv", ["p0", "p1"], np.ones((2, 6)), extra_columns=False)
    motions, note = check_captures.read_joint_motions(log, {"p0", "p1"})
    assert motions == {} and "joint-sign report is skipped" in note


def test_joint_sign_report_through_main(session, tmp_path, capsys):
    pose_ids = list(session["poses"])
    count = len(pose_ids)
    motions = np.ones((count, 6))
    motions[: count // 2, 0] = -1.0           # joint 1: half the poses move each way -> warned
    log = write_pose_log(tmp_path / "pose_log.csv", pose_ids, motions)
    report = tmp_path / "check.json"
    code = check_captures.main(["--manifest", str(session["manifest"]), "--pose-log", str(log), "--out", str(report)])
    printed = capsys.readouterr()
    assert code == EXIT_OK                    # the joint-sign report is informational
    assert "Joint-sign report" in printed.out
    assert "WARNING: pose log" in printed.err and "joint j1" in printed.err and "joint j2" not in printed.err
    joint_rows = {row["joint"]: row for row in json.loads(report.read_text())["joint_sign"]}
    assert joint_rows[2]["majority_fraction"] == 1.0
    assert np.isclose(joint_rows[1]["majority_fraction"], max(count // 2, count - count // 2) / count)
    # Without the columns the note is printed and the JSON has no joint report.
    bare = write_pose_log(tmp_path / "bare.csv", pose_ids, motions, extra_columns=False)
    assert check_captures.main(["--manifest", str(session["manifest"]), "--pose-log", str(bare), "--out", str(report)]) == EXIT_OK
    assert "NOTE:" in capsys.readouterr().out
    assert json.loads(report.read_text())["joint_sign"] is None
