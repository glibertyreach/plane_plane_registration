"""
Registration of a capture session: the sensor-to-base transform from the board poses, with figures.

    python3 -m planereg.analysis.register --manifest manifest.json --out DIR
        [--sensor-in-base PATH] [--model rigid|similarity|both] [--outlier-rounds N]
        [--target-offset-mm D] [--figures | --no-figures] [--truth truth.json]

Two passes (code design 8.1), both through the shared pipeline of ``planereg.core.pipeline``:

    pass 1  every pose is segmented by the closest large plane (or, with --sensor-in-base, by the predicted
            region of that rough transform) and the poses are registered;
    pass 2  every pose is segmented again with the prediction from the pass-1 transform of the same model
            and the poses are registered again. This is the result.

When pass 1 does not solve (too few poses, degenerate normals) pass 2 is skipped for that model and pass 1
stands as the final result; the document says so.

For the similarity model the pass-2 prediction uses the pass-1 rotation, translation and scale (the core's
prediction is scale-aware). The plane itself is always fitted to the measured points, never taken from the
prediction.

Outputs in --out
    registration.json    parameters; per model and pass: transform (4 x 4, rotation, translation, scale),
                         status, spreads, RMS and maximum residuals, rejected poses and per pose the
                         segmentation statistics and residuals; optionally the errors against a truth file
    segmentation.npz     the mask of every pose, key "<model>_pass<n>__<pose_id>" (the masks differ per model
                         and pass, so a bare pose id would be ambiguous)
    figures/<model>/     the figures of ``planereg.analysis.residual_maps`` when enabled

Exit codes: 0 every requested model passed its acceptance thresholds, 1 a model was solved but flagged or
could not be solved, 2 an input must be fixed.

Frames and units: sensor S, base B; the transform is X: S -> B, X = [c R, s; 0 1]; millimeters, degrees.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass, field, replace
from enum import Enum
from pathlib import Path

import numpy as np

from planereg.core.pipeline import PipelineParameters, PoseMeasurement, measure_all, register_measurements
from planereg.core.registration import (RegistrationParameters, RegistrationResult, RegistrationStatus,
                                        TransformModel)
from planereg.core.segmentation import SegmentationParameters
from sphcal.cli.plan_poses import PlanInputError, load_sensor_in_base
from sphcal.geometry.transforms import RigidTransform
from sphcal.io.capture_set import CaptureSet
from sphcal.io.poses import TARGET_KIND_BOARD, load_manifest

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
EXIT_OK = 0
EXIT_FLAGGED = 1
EXIT_INPUT_ERROR = 2
"""Exit codes: every model accepted; a model flagged or unsolved; an input to fix."""

SCHEMA_VERSION = 1
"""Version of the registration.json layout (readers by compare and report check it)."""
REGISTRATION_FILE_NAME = "registration.json"
SEGMENTATION_FILE_NAME = "segmentation.npz"
FIGURE_DIRECTORY_NAME = "figures"
"""File and directory names written into --out."""

PASS_ONE = "pass1"
PASS_TWO = "pass2"
"""Names of the two passes in the document."""
MASK_KEY_FORMAT = "{model}_{pass_name}__{pose_id}"
"""Key of a pose's mask in segmentation.npz."""

MODEL_RIGID = TransformModel.RIGID.value
MODEL_SIMILARITY = TransformModel.SIMILARITY.value
MODEL_BOTH = "both"
MODEL_CHOICES = (MODEL_RIGID, MODEL_SIMILARITY, MODEL_BOTH)
"""Command-line choices of --model."""

BOOTSTRAP_RESIDUAL_WARN_MM = float("inf")
"""Residual limit passed to sphcal's loader of --sensor-in-base; this tool does not judge a bootstrap."""
PREDICTION_NONE = "none"
PREDICTION_SENSOR_IN_BASE = "sensor_in_base"
PREDICTION_PASS_ONE = "pass1_transform"
"""How a pass found the candidate region of each pose."""

POSE_COLUMN_WIDTH = 30
"""Width of the pose-id column of the console table."""
TRUTH_RIGID_KEY = "sensor_to_base"
"""Key of the true transform in truth.json (written by planereg.analysis.simulate)."""


# ---------------------------------------------------------------------------
# JSON helpers (shared by the other analysis tools)
# ---------------------------------------------------------------------------
def json_safe(value):
    """Convert numpy values, enums and paths to JSON types; NaN and infinity become null."""
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, (np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def load_registration(path: Path) -> dict:
    """Read a registration.json. Raises ValueError, saying what is wrong, for a file that is not one."""
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"cannot read {path}: {error.strerror}") from None
    except json.JSONDecodeError as error:
        raise ValueError(f"{path} is not valid JSON ({error})") from None
    if not isinstance(document, dict) or document.get("schema_version") != SCHEMA_VERSION or "models" not in document:
        raise ValueError(f"{path} is not a registration.json of schema version {SCHEMA_VERSION}")
    return document


def final_block(model_entry: dict) -> dict:
    """The result block of a model that counts: pass 2 when it was run, else pass 1."""
    return model_entry[model_entry["final_pass"]]


def transform_from_block(block: dict) -> tuple[RigidTransform, float]:
    """(rigid part X = (R, s), scale c) of a result block; raises ValueError when the block holds no solution."""
    if not block.get("solved"):
        raise ValueError("the registration was not solved")
    return RigidTransform(np.array(block["rotation"]), np.array(block["translation_mm"])), float(block["scale"])


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class RegisterParameters:
    """Everything the registration run takes from the command line."""

    models: tuple[str, ...] = (MODEL_RIGID, MODEL_SIMILARITY)
    pipeline: PipelineParameters = field(default_factory=PipelineParameters)
    registration: RegistrationParameters = field(default_factory=RegistrationParameters)
    """Thresholds and outlier rejection; the transform model is set per model run."""


@dataclass
class PassRun:
    """One registration pass of one model."""

    name: str                                   # PASS_ONE or PASS_TWO
    prediction: str                             # how the candidate regions were found
    measurements: list[PoseMeasurement]
    result: RegistrationResult
    pose_ids: list[str]                         # poses in the registration, in the order of result.residuals


@dataclass
class ModelRun:
    """Both passes of one transform model."""

    model: str
    registration_parameters: RegistrationParameters
    pass_one: PassRun
    pass_two: PassRun | None

    def final(self) -> PassRun:
        return self.pass_two if self.pass_two is not None else self.pass_one


# ---------------------------------------------------------------------------
# The two passes
# ---------------------------------------------------------------------------
def model_parameters(params: RegisterParameters, model: str) -> RegistrationParameters:
    """The registration parameters with the transform model of ``model``."""
    return replace(params.registration, transform_model=TransformModel(model))


def run_model(capture_set: CaptureSet, params: RegisterParameters, model: str,
              pass_one_measurements: list[PoseMeasurement], pass_one_prediction: str) -> ModelRun:
    """Register the pass-1 measurements with ``model``; if that solves, segment every pose again with the
    prediction from the solution and register again."""
    registration_parameters = model_parameters(params, model)
    result_one, ids_one = register_measurements(pass_one_measurements, registration_parameters)
    pass_one = PassRun(PASS_ONE, pass_one_prediction, pass_one_measurements, result_one, ids_one)
    if not result_one.solved():
        return ModelRun(model, registration_parameters, pass_one, None)
    measurements_two = measure_all(capture_set, params.pipeline, result_one.sensor_to_base, result_one.scale)
    result_two, ids_two = register_measurements(measurements_two, registration_parameters)
    pass_two = PassRun(PASS_TWO, PREDICTION_PASS_ONE, measurements_two, result_two, ids_two)
    return ModelRun(model, registration_parameters, pass_one, pass_two)


def register_session(capture_set: CaptureSet, params: RegisterParameters,
                     rough_sensor_to_base: RigidTransform | None = None) -> list[ModelRun]:
    """Pass 1 once (it does not depend on the model), then both passes' registration per model."""
    pass_one_measurements = measure_all(capture_set, params.pipeline, rough_sensor_to_base)
    prediction = PREDICTION_NONE if rough_sensor_to_base is None else PREDICTION_SENSOR_IN_BASE
    return [run_model(capture_set, params, model, pass_one_measurements, prediction) for model in params.models]


# ---------------------------------------------------------------------------
# Document
# ---------------------------------------------------------------------------
def pose_entries(run: PassRun) -> list[dict]:
    """One entry per pose of the pass (all poses, segmented or not): segmentation statistics, board center,
    and, for poses that entered the solve, the residuals. ``normal_residual_sensor`` is the normal residual
    vector R^t (R n_k - m_k) = n_k - R^t m_k in the sensor frame (measured minus predicted normal), whose x
    and y components are along the image's u and v axes."""
    residual_of = dict(zip(run.pose_ids, run.result.residuals))
    rotation = run.result.sensor_to_base.rotation
    entries = []
    for measurement in run.measurements:
        segmentation = measurement.segmentation
        entry = {
            "pose_id": measurement.pose_id, "kind": measurement.kind, "frames": measurement.frames,
            "valid_fraction": measurement.valid_fraction,
            "segmented": measurement.ok, "error": measurement.error,
            "method": None if segmentation is None else segmentation.method,
            "message": None if segmentation is None else segmentation.message,
            "pixels": 0 if segmentation is None else segmentation.inlier_count,
            "plane_rms_mm": float("nan") if segmentation is None else segmentation.rms_mm,
            "refinement_rounds": 0 if segmentation is None else segmentation.rounds,
            "edge_shrink_px": 0 if segmentation is None else segmentation.edge_shrink_px,
            "candidate_areas_mm2": [] if segmentation is None else segmentation.candidate_areas_mm2,
            "board_center_uv": list(measurement.board_center_uv),
            "board_center_sensor_mm": measurement.board_center_sensor_mm,
            "touches_border": measurement.touches_border,
            "used": False, "normal_residual_deg": float("nan"), "offset_residual_mm": float("nan"),
            "normal_residual_sensor": [float("nan")] * 3,
        }
        residual = residual_of.get(measurement.pose_id)
        if residual is not None and measurement.ok:
            entry["used"] = bool(residual.used)
            entry["normal_residual_deg"] = residual.normal_angle_degrees
            entry["offset_residual_mm"] = residual.offset_residual_mm
            entry["normal_residual_sensor"] = (measurement.sensor_plane.normal
                                               - rotation.T @ measurement.base_plane.normal)
        entries.append(entry)
    return entries


def pass_block(run: PassRun) -> dict:
    """The result block of one pass (see the module docstring and residual_maps / compare / report)."""
    result = run.result
    block = {
        "prediction": run.prediction,
        "status": result.status.value, "message": result.message, "solved": result.solved(),
        "accepted": result.status == RegistrationStatus.SUCCESS,
        "poses_total": len(run.measurements), "poses_segmented": sum(m.ok for m in run.measurements),
        "poses_used": result.poses_used,
        "normal_spread": result.normal_spread, "similarity_spread": result.similarity_spread,
        "rms_normal_residual_deg": result.rms_normal_residual_degrees,
        "max_normal_residual_deg": result.max_normal_residual_degrees,
        "rms_offset_residual_mm": result.rms_offset_residual_mm,
        "max_offset_residual_mm": result.max_offset_residual_mm,
        "matrix": None, "rotation": None, "rotation_vector_deg": None, "translation_mm": None, "scale": None,
        "rejected_pose_ids": [pid for pid, r in zip(run.pose_ids, result.residuals) if not r.used],
        "poses": pose_entries(run),
    }
    if result.solved():
        block.update(matrix=result.sensor_to_base_matrix(), rotation=result.sensor_to_base.rotation,
                     rotation_vector_deg=result.sensor_to_base.rotation_vector_degrees(),
                     translation_mm=result.sensor_to_base.translation, scale=result.scale)
    return block


def registration_document(manifest_path: Path, params: RegisterParameters, runs: list[ModelRun],
                          image_size_px: tuple[int, int], rough_path: Path | None, outlier_rounds: int) -> dict:
    """The registration.json dictionary."""
    models = {}
    for run in runs:
        entry = {PASS_ONE: pass_block(run.pass_one), "final_pass": run.final().name}
        entry[PASS_TWO] = None if run.pass_two is None else pass_block(run.pass_two)
        models[run.model] = entry
    return json_safe({
        "schema_version": SCHEMA_VERSION,
        "manifest": Path(manifest_path).resolve(),
        "sensor_in_base": None if rough_path is None else Path(rough_path).resolve(),
        "segmentation_file": SEGMENTATION_FILE_NAME,
        "image_size_px": list(image_size_px),
        "parameters": {
            "pipeline": asdict(params.pipeline),
            "registration": {**asdict(params.registration), "outlier_rejection_rounds": outlier_rounds},
        },
        "model_order": [run.model for run in runs],
        "models": models,
    })


def segmentation_masks(runs: list[ModelRun]) -> dict[str, np.ndarray]:
    """Every pose's mask of every model and pass, keyed by MASK_KEY_FORMAT."""
    masks = {}
    for run in runs:
        for pass_run in (run.pass_one, run.pass_two):
            if pass_run is None:
                continue
            for measurement in pass_run.measurements:
                if measurement.segmentation is not None:
                    masks[MASK_KEY_FORMAT.format(model=run.model, pass_name=pass_run.name,
                                                 pose_id=measurement.pose_id)] = measurement.segmentation.mask
    return masks


# ---------------------------------------------------------------------------
# Comparison with a known truth (simulated sessions)
# ---------------------------------------------------------------------------
def error_against_truth(block: dict, truth: dict) -> dict | None:
    """Errors of a solved result block against the transform in a truth.json dictionary: rotation angle (deg),
    translation distance (mm) between the rigid parts, and the difference of the scales."""
    if not block.get("solved"):
        return None
    true_entry = truth[TRUTH_RIGID_KEY]
    true_rigid = RigidTransform(np.array(true_entry["rotation"]), np.array(true_entry["translation_mm"]))
    estimated, scale = transform_from_block(block)
    translation_error, rotation_error = estimated.difference_from(true_rigid)
    return {"rotation_error_deg": rotation_error, "translation_error_mm": translation_error,
            "scale": scale, "scale_error": scale - float(true_entry["scale"])}


def truth_comparison(document: dict, truth: dict) -> dict:
    """{model: {pass: errors}} for every solved pass of the document."""
    comparison = {}
    for model, entry in document["models"].items():
        comparison[model] = {}
        for pass_name in (PASS_ONE, PASS_TWO):
            if entry[pass_name] is not None:
                comparison[model][pass_name] = error_against_truth(entry[pass_name], truth)
    return comparison


# ---------------------------------------------------------------------------
# Console output
# ---------------------------------------------------------------------------
def print_summary(runs: list[ModelRun], comparison: dict | None) -> None:
    """One line per model and pass, then the per-pose table of the final pass of each model."""
    for run in runs:
        for pass_run in (run.pass_one, run.pass_two):
            if pass_run is None:
                continue
            result = pass_run.result
            line = (f"{run.model:<10} {pass_run.name}: {result.status.value}, "
                    f"{result.poses_used}/{len(pass_run.measurements)} poses used, "
                    f"RMS normal {result.rms_normal_residual_degrees:.3f} deg, "
                    f"RMS offset {result.rms_offset_residual_mm:.3f} mm, scale {result.scale:.5f}")
            truth_errors = None if comparison is None else comparison[run.model].get(pass_run.name)
            if truth_errors is not None:
                line += (f"; vs truth {truth_errors['rotation_error_deg']:.4f} deg, "
                         f"{truth_errors['translation_error_mm']:.3f} mm")
            print(line)
            if not result.solved():
                print(f"WARNING: {run.model} {pass_run.name}: {result.message}")
    for run in runs:
        final = run.final()
        print(f"\npose table, {run.model} {final.name}")
        print(f"{'pose':<{POSE_COLUMN_WIDTH}} {'method':<20} {'pixels':>7} {'rms mm':>7} {'normal deg':>10} "
              f"{'offset mm':>10}  used")
        for entry in pose_entries(final):
            print(f"{entry['pose_id']:<{POSE_COLUMN_WIDTH}} {str(entry['method']):<20} {entry['pixels']:>7} "
                  f"{entry['plane_rms_mm']:>7.3f} {entry['normal_residual_deg']:>10.3f} "
                  f"{entry['offset_residual_mm']:>10.3f}  {'yes' if entry['used'] else 'NO'}")


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    pipeline = PipelineParameters()
    registration = RegistrationParameters()
    parser = argparse.ArgumentParser(
        prog="python3 -m planereg.analysis.register",
        description="Register a capture session: segment every board, solve the sensor-to-base transform "
                    "in two passes, write registration.json, segmentation.npz and figures.")
    parser.add_argument("--manifest", required=True, type=Path, help="capture manifest (JSON or CSV)")
    parser.add_argument("--out", required=True, type=Path, help="output directory (created)")
    parser.add_argument("--sensor-in-base", type=Path, default=None,
                        help="rough sensor-to-base JSON ({\"matrix\": [...]}); pass 1 then uses the predicted region")
    parser.add_argument("--model", choices=MODEL_CHOICES, default=MODEL_BOTH, help="transform model(s) to solve")
    parser.add_argument("--outlier-rounds", type=int, default=registration.outlier_rejection_rounds,
                        help="poses that may be dropped, one per round, when a residual exceeds its outlier limit")
    parser.add_argument("--outlier-normal-deg", type=float, default=registration.outlier_normal_residual_degrees)
    parser.add_argument("--outlier-offset-mm", type=float, default=registration.outlier_offset_residual_mm)
    parser.add_argument("--target-offset-mm", type=float, default=pipeline.target_offset_mm,
                        help="distance of the board face along the logged frame's +z (0 for the board tool frame)")
    parser.add_argument("--min-valid-fraction", type=float, default=pipeline.min_valid_fraction)
    parser.add_argument("--max-rms-normal-deg", type=float, default=registration.maximum_rms_normal_residual_degrees)
    parser.add_argument("--max-rms-offset-mm", type=float, default=registration.maximum_rms_offset_residual_mm)
    parser.add_argument("--max-normal-deg", type=float, default=registration.maximum_normal_residual_degrees)
    parser.add_argument("--max-offset-mm", type=float, default=registration.maximum_offset_residual_mm)
    parser.add_argument("--min-normal-spread", type=float, default=registration.minimum_normal_spread)
    parser.add_argument("--min-similarity-spread", type=float, default=registration.minimum_similarity_spread)
    parser.add_argument("--figures", action=argparse.BooleanOptionalAction, default=True,
                        help="write the residual figures (needs matplotlib)")
    parser.add_argument("--pixel-maps", type=int, default=None,
                        help="per-pixel residual maps of the N poses with the largest offset residual "
                             "(default of residual_maps)")
    parser.add_argument("--pixel-maps-random", type=int, default=None,
                        help="per-pixel maps of this many further, randomly chosen poses (default of residual_maps)")
    parser.add_argument("--truth", type=Path, default=None,
                        help="truth.json of a simulated session: adds the errors against the true transform")
    return parser


def parameters_from_arguments(args: argparse.Namespace) -> RegisterParameters:
    models = (MODEL_RIGID, MODEL_SIMILARITY) if args.model == MODEL_BOTH else (args.model,)
    pipeline = PipelineParameters(min_valid_fraction=args.min_valid_fraction, segmentation=SegmentationParameters(),
                                  target_offset_mm=args.target_offset_mm)
    registration = RegistrationParameters(
        outlier_rejection_rounds=args.outlier_rounds, outlier_normal_residual_degrees=args.outlier_normal_deg,
        outlier_offset_residual_mm=args.outlier_offset_mm,
        maximum_rms_normal_residual_degrees=args.max_rms_normal_deg,
        maximum_rms_offset_residual_mm=args.max_rms_offset_mm,
        maximum_normal_residual_degrees=args.max_normal_deg, maximum_offset_residual_mm=args.max_offset_mm,
        minimum_normal_spread=args.min_normal_spread, minimum_similarity_spread=args.min_similarity_spread)
    return RegisterParameters(models, pipeline, registration)


def load_board_records(manifest_path: Path) -> CaptureSet:
    """The manifest's records of boards as a CaptureSet. Raises ValueError, saying what to fix."""
    try:
        records = load_manifest(manifest_path)
    except OSError as error:
        raise ValueError(f"cannot read the manifest {manifest_path}: {error.strerror}") from None
    boards = [record for record in records if record.target_kind == TARGET_KIND_BOARD]
    if len(boards) < len(records):
        print(f"WARNING: {len(records) - len(boards)} manifest record(s) are not boards and are ignored")
    if not boards:
        raise ValueError(f"the manifest {manifest_path} holds no board capture")
    return CaptureSet(boards)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    params = parameters_from_arguments(args)
    try:
        capture_set = load_board_records(args.manifest)
        rough = None
        if args.sensor_in_base is not None:
            rough, _, _ = load_sensor_in_base(args.sensor_in_base, BOOTSTRAP_RESIDUAL_WARN_MM)
        truth = None if args.truth is None else json.loads(args.truth.read_text(encoding="utf-8"))
    except (ValueError, PlanInputError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return EXIT_INPUT_ERROR

    runs = register_session(capture_set, params, rough)
    first_mask = next((m.segmentation.mask for m in runs[0].pass_one.measurements if m.segmentation is not None), None)
    image_size = (0, 0) if first_mask is None else (first_mask.shape[1], first_mask.shape[0])
    document = registration_document(args.manifest, params, runs, image_size, args.sensor_in_base,
                                     params.registration.outlier_rejection_rounds)
    comparison = None
    if truth is not None:
        comparison = json_safe(truth_comparison(document, truth))
        document["truth_comparison"] = comparison

    args.out.mkdir(parents=True, exist_ok=True)
    masks = segmentation_masks(runs)
    np.savez_compressed(args.out / SEGMENTATION_FILE_NAME, **masks)
    (args.out / REGISTRATION_FILE_NAME).write_text(json.dumps(document, indent=2), encoding="utf-8")
    print_summary(runs, comparison)

    if args.figures:
        from planereg.analysis import residual_maps    # lazy: matplotlib is optional and the module imports this one
        defaults = residual_maps.FigureParameters()
        figure_parameters = replace(
            defaults,
            pixel_map_count=defaults.pixel_map_count if args.pixel_maps is None else args.pixel_maps,
            pixel_map_random_count=(defaults.pixel_map_random_count if args.pixel_maps_random is None
                                    else args.pixel_maps_random))
        for model in document["model_order"]:
            written = residual_maps.make_figures(document, masks, args.out / FIGURE_DIRECTORY_NAME / model, model,
                                                 figure_parameters)
            print(f"wrote {len(written)} figure(s) for {model} to {args.out / FIGURE_DIRECTORY_NAME / model}")
    print(f"wrote {args.out / REGISTRATION_FILE_NAME}")
    flagged = any(not run.final().result.status == RegistrationStatus.SUCCESS for run in runs)
    return EXIT_FLAGGED if flagged else EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
