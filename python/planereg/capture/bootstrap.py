"""
Command line: find the sensor roughly in the robot base frame from a few hand-jogged board captures
(code design, bootstrap tool).

    python3 -m planereg.capture.bootstrap --manifest manifest.json --out sensor_in_base.json \
        [--target-offset-mm D]

What it does
------------
The technician jogs the board to four or more poses spread through the field of view, tilting it
differently at each, captures them, and builds a manifest with ``sphcal.cli.make_manifest``. This
tool then

    1. segments the board in every pose WITHOUT a prediction (the closest large plane,
       ``planereg.core.pipeline.measure_all``),
    2. registers the poses with the rigid model (``register_measurements``): the rotation from the
       plane normals, the translation from the plane offsets,
    3. prints a table (pose, segmentation method, pixels, plane RMS, normal residual, offset
       residual) and the transform, and
    4. writes a JSON file::

           {"matrix": [16 floats, row-major sensor-to-base], "status": "Success", "message": "...",
            "normal_spread": ..., "rms_normal_residual_deg": ..., "rms_offset_residual_mm": ...,
            "poses": [{"pose_id", "method", "pixels", "plane_rms_mm", "normal_residual_deg",
                       "offset_residual_mm", "used", "message"}, ...]}

       ``matrix`` is what ``sphcal.cli.plan_poses --sensor-in-base`` and
       ``planereg.capture.plan_poses --sensor-in-base`` read (they ignore the other keys, except that
       planereg's plan tool warns when ``status`` is not Success). ``matrix`` is null when the
       registration could not be solved at all.

The thresholds here are looser than the registration defaults on purpose: the bootstrap transform
only has to be good enough to PREDICT where the board is in later captures, and the predicted region
of the segmentation tolerates about 2 degrees and 20 mm (code design, Section 9). Every threshold
is a command-line option.

Logged frame. The manifest's pose is normally the board tool frame (origin at the center of the board's
front face). If the controller can only report the flange pose, log that and pass ``--target-offset-mm D``
with D the distance from the flange origin to the board's front face along the flange +z (the plate
thickness); every logged plane is then shifted by D along the logged frame's +z.

Frames: the sensor frame S (the frame of the points in a capture) and the base frame B; the manifest's
target pose is the board tool frame T -> B. Units: millimeters, degrees.

Exit code: 0 when the registration status is Success, 1 when it is not (with advice on what to do),
2 when the manifest cannot be read or holds no board.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from planereg.capture.common import (EXIT_FLAGGED, EXIT_INPUT_ERROR, EXIT_OK, MATRIX_PRINT_DECIMALS,
                                     STATUS_SUCCESS_TEXT, board_poses_only, board_touches_border, format_number, print_error,
                                     print_warning, write_json)
from planereg.core.pipeline import DEFAULT_BORDER_MARGIN_PX, PipelineParameters, PoseMeasurement, measure_all, \
    register_measurements
from planereg.core.registration import RegistrationParameters, RegistrationResult, RegistrationStatus, \
    TransformModel
from sphcal.io.capture_set import CaptureSet
from sphcal.io.poses import load_manifest

MINIMUM_BOOTSTRAP_POSES = 4
"""Fewest board poses the bootstrap accepts (the design asks for four or more hand-jogged poses)."""
VECTOR_PRINT_DECIMALS = 2
"""Decimals printed for the rotation vector and the translation of the solved transform."""
ADVICE_TOO_FEW_POSES = ("capture more board poses: at least {required} are needed, and every pose must show the "
                        "whole board with the closest large flat surface being the board")
ADVICE_DEGENERATE = ("tilt the board more between poses (different tilt directions, not only different positions) "
                     "and capture at least two poses at clearly different distances")
ADVICE_RESIDUALS = ("re-jog the pose with the largest residual ({worst}), and check the tool frame: the board "
                    "tool frame must be active and its origin and z axis must be the center and normal of the "
                    "board's front face")
ADVICE_NO_SEGMENTATION = ("the board could not be found in any pose; check that the board is the closest large "
                          "flat surface in view and fills enough of the image")


@dataclass(frozen=True)
class BootstrapParameters:
    """Thresholds of the bootstrap; the command-line defaults come from here."""

    min_valid_fraction: float = 0.5
    """A pixel enters the temporal mean only if valid in at least this fraction of the frames."""
    border_margin_px: int = DEFAULT_BORDER_MARGIN_PX
    """A board whose mask comes this close to the image border is reported as partly out of view."""
    target_offset_mm: float = 0.0
    """Distance from the logged frame's origin to the board's front face along the logged frame's +z, in mm:
    0 when the logged pose is the board tool frame, the flange-to-board-face distance D when the controller
    can only report the flange pose (``PipelineParameters.target_offset_mm``)."""
    minimum_pose_count: int = MINIMUM_BOOTSTRAP_POSES
    """Fewest segmented poses the registration may use."""
    minimum_normal_spread: float = 0.05
    """Gate on the spread of the logged board normals (RegistrationParameters default)."""
    max_rms_normal_residual_deg: float = 1.0
    max_rms_offset_residual_mm: float = 5.0
    """RMS residual limits over the poses."""
    max_normal_residual_deg: float = 2.0
    max_offset_residual_mm: float = 10.0
    """Limits on the worst single pose."""

    def registration_parameters(self) -> RegistrationParameters:
        """The rigid registration with these thresholds and no outlier rejection (every pose is shown)."""
        return RegistrationParameters(
            transform_model=TransformModel.RIGID, minimum_pose_count=self.minimum_pose_count,
            minimum_normal_spread=self.minimum_normal_spread,
            maximum_rms_normal_residual_degrees=self.max_rms_normal_residual_deg,
            maximum_rms_offset_residual_mm=self.max_rms_offset_residual_mm,
            maximum_normal_residual_degrees=self.max_normal_residual_deg,
            maximum_offset_residual_mm=self.max_offset_residual_mm, outlier_rejection_rounds=0)


# ---------------------------------------------------------------------------
# Per-pose rows
# ---------------------------------------------------------------------------
def pose_rows(measurements: list[PoseMeasurement], result: RegistrationResult, used_ids: list[str],
              border_margin_px: int) -> list[dict]:
    """One dictionary per pose in manifest order: segmentation statistics, and the residuals for the poses
    that took part in the registration (NaN for the others, with the reason in ``message``)."""
    residual_of = dict(zip(used_ids, result.residuals)) if result.solved() else {}
    rows = []
    for m in measurements:
        residual = residual_of.get(m.pose_id)
        segmentation = m.segmentation
        rows.append({
            "pose_id": m.pose_id,
            "method": segmentation.method if segmentation is not None else "",
            "pixels": segmentation.inlier_count if segmentation is not None else 0,
            "plane_rms_mm": segmentation.rms_mm if segmentation is not None else float("nan"),
            "normal_residual_deg": residual.normal_angle_degrees if residual else float("nan"),
            "offset_residual_mm": residual.offset_residual_mm if residual else float("nan"),
            "used": bool(residual.used) if residual else False,
            "touches_border": board_touches_border(m, border_margin_px),
            "message": m.error})
    return rows


def format_table(rows: list[dict]) -> str:
    """Console table of all poses."""
    width = max([len("pose_id")] + [len(row["pose_id"]) for row in rows])
    header = (f"{'pose_id':<{width}}  {'method':<20} {'pixels':>7} {'plane_rms_mm':>12} {'normal_deg':>10} "
              f"{'offset_mm':>10}  note")
    lines = [header, "-" * len(header)]
    for row in rows:
        note = row["message"] or ("not used" if not row["used"] else "ok")
        lines.append(f"{row['pose_id']:<{width}}  {row['method'] or '-':<20} {row['pixels']:>7d} "
                     f"{format_number(row['plane_rms_mm'], '.3f'):>12} "
                     f"{format_number(row['normal_residual_deg'], '.3f'):>10} "
                     f"{format_number(row['offset_residual_mm'], '+.3f'):>10}  {note}")
    return "\n".join(lines)


def advice_for(result: RegistrationResult, rows: list[dict], params: BootstrapParameters) -> str:
    """What the technician should do about a registration that did not succeed."""
    if result.status == RegistrationStatus.TOO_FEW_POSES:
        return (ADVICE_NO_SEGMENTATION if not any(row["pixels"] for row in rows)
                else ADVICE_TOO_FEW_POSES.format(required=params.minimum_pose_count))
    if result.status == RegistrationStatus.DEGENERATE_POSES:
        return ADVICE_DEGENERATE
    if result.status == RegistrationStatus.RESIDUALS_EXCEED_THRESHOLD:
        used = [row for row in rows if row["used"]]
        worst = max(used, key=lambda row: max(row["normal_residual_deg"] / params.max_normal_residual_deg,
                                              abs(row["offset_residual_mm"]) / params.max_offset_residual_mm))
        return ADVICE_RESIDUALS.format(worst=worst["pose_id"])
    return "check the manifest and the pose log"


def report_dictionary(result: RegistrationResult, rows: list[dict]) -> dict:
    """The JSON document (module docstring)."""
    return {
        "matrix": result.sensor_to_base_matrix().reshape(-1).tolist() if result.solved() else None,
        "status": result.status.value,
        "message": result.message,
        "normal_spread": result.normal_spread,
        "rms_normal_residual_deg": result.rms_normal_residual_degrees,
        "rms_offset_residual_mm": result.rms_offset_residual_mm,
        "poses": rows}


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    d = BootstrapParameters()
    parser = argparse.ArgumentParser(
        description="Find the sensor roughly in the robot base frame from four or more hand-jogged board captures "
                    "(manifest made by sphcal's make_manifest): segment the board in each, register, print the "
                    "table and the transform, and write the JSON that the plan tool reads.")
    parser.add_argument("--manifest", required=True, type=Path, metavar="PATH", help="manifest CSV or JSON")
    parser.add_argument("--out", required=True, type=Path, metavar="PATH", help="JSON file to write")
    parser.add_argument("--min-valid-fraction", type=float, default=d.min_valid_fraction,
                        help="a pixel must be valid in this fraction of a pose's frames (default %(default)s)")
    parser.add_argument("--border-margin-px", type=int, default=d.border_margin_px,
                        help="warn when the board mask comes this close to the image border (default %(default)s)")
    parser.add_argument("--target-offset-mm", type=float, default=d.target_offset_mm,
                        help="distance from the logged frame's origin to the board's front face along its +z, in mm; "
                             "0 when the logged frame is the board tool frame, the flange-face-to-front-face distance D "
                             "of the procedure when the flange pose was logged (default %(default)s)")
    parser.add_argument("--min-pose-count", type=int, default=d.minimum_pose_count,
                        help="fewest segmented poses the registration may use (default %(default)s)")
    parser.add_argument("--min-normal-spread", type=float, default=d.minimum_normal_spread,
                        help="smallest accepted spread of the board normals (default %(default)s)")
    parser.add_argument("--max-rms-normal-deg", type=float, default=d.max_rms_normal_residual_deg,
                        help="largest accepted RMS normal residual (default %(default)s)")
    parser.add_argument("--max-rms-offset-mm", type=float, default=d.max_rms_offset_residual_mm,
                        help="largest accepted RMS offset residual (default %(default)s)")
    parser.add_argument("--max-normal-deg", type=float, default=d.max_normal_residual_deg,
                        help="largest accepted normal residual of a single pose (default %(default)s)")
    parser.add_argument("--max-offset-mm", type=float, default=d.max_offset_residual_mm,
                        help="largest accepted offset residual of a single pose (default %(default)s)")
    return parser


def parameters_from_arguments(args: argparse.Namespace) -> BootstrapParameters:
    """The BootstrapParameters of the parsed command line."""
    return BootstrapParameters(
        min_valid_fraction=args.min_valid_fraction, border_margin_px=args.border_margin_px,
        target_offset_mm=args.target_offset_mm, minimum_pose_count=args.min_pose_count, minimum_normal_spread=args.min_normal_spread,
        max_rms_normal_residual_deg=args.max_rms_normal_deg, max_rms_offset_residual_mm=args.max_rms_offset_mm,
        max_normal_residual_deg=args.max_normal_deg, max_offset_residual_mm=args.max_offset_mm)


def main(argv: list[str] | None = None) -> int:
    """Run the tool; returns the exit code (0 Success, 1 not Success, 2 input must be fixed)."""
    args = build_parser().parse_args(argv)
    params = parameters_from_arguments(args)
    try:
        capture_set = CaptureSet(load_manifest(args.manifest))
    except (OSError, ValueError) as error:
        print_error(f"cannot read the manifest {args.manifest}: {error}. Fix the manifest (make_manifest writes a "
                    "valid one) and run again.")
        return EXIT_INPUT_ERROR
    capture_set, skipped = board_poses_only(capture_set)
    for message in skipped:
        print(message, file=sys.stderr)
    if not capture_set.records:
        print_error(f"the manifest {args.manifest} lists no board captures.")
        return EXIT_INPUT_ERROR
    pipeline = PipelineParameters(min_valid_fraction=params.min_valid_fraction, border_margin_px=params.border_margin_px,
                                  target_offset_mm=params.target_offset_mm)
    measurements = measure_all(capture_set, pipeline)          # no prediction: the closest large plane
    for m in measurements:
        if not m.ok:
            print_warning(f"pose {m.pose_id}: {m.error}; the pose is left out of the registration.")
        elif board_touches_border(m, params.border_margin_px):
            print_warning(f"pose {m.pose_id}: the board touches the image border (board partly out of view); "
                          "the pose was still used, but re-capture it with the board fully in view.")
    result, used_ids = register_measurements(measurements, params.registration_parameters())
    rows = pose_rows(measurements, result, used_ids, params.border_margin_px)
    print(format_table(rows))
    if result.solved():
        print("Sensor-to-base transform (row-major 4x4, mm):")
        print(np.array2string(result.sensor_to_base_matrix(), precision=MATRIX_PRINT_DECIMALS, suppress_small=True))
        rotation_vector = result.sensor_to_base.rotation_vector_degrees()
        print(f"Rotation vector {np.array2string(rotation_vector, precision=VECTOR_PRINT_DECIMALS)} deg; "
              f"translation {np.array2string(result.sensor_to_base.translation, precision=VECTOR_PRINT_DECIMALS)} mm")
        print(f"Poses used {result.poses_used}; normal spread {result.normal_spread:.4f}; RMS residuals "
              f"{result.rms_normal_residual_degrees:.3f} deg and {result.rms_offset_residual_mm:.3f} mm")
    print(f"Registration status: {result.status.value}: {result.message}")
    write_json(args.out, report_dictionary(result, rows))
    print(f"Wrote {args.out}")
    if result.status.value == STATUS_SUCCESS_TEXT:
        return EXIT_OK
    print_error(f"the bootstrap registration is not Success ({result.status.value}); do not plan with "
                f"{args.out}. Advice: {advice_for(result, rows, params)}.")
    return EXIT_FLAGGED


if __name__ == "__main__":
    sys.exit(main())
