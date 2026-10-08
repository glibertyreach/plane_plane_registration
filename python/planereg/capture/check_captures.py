"""
Command line: a quick check of a capture set before the long registration (code design, check tool).

    python3 -m planereg.capture.check_captures --manifest manifest.json \
        [--sensor-in-base sensor_in_base.json] [--pose-log pose_log.csv] [--target-offset-mm D] \
        --out check.json

What it checks, per pose (the frames of one commanded pose)
    frames          number of capture files of the pose.
    valid fraction  temporal read rate: over the pixels valid in at least one frame, the mean
                    fraction of frames in which they are valid; a board that flickers in and out
                    scores low.
    segmentation    ``planereg.core.pipeline.measure_pose``: the board's plane is found with the
                    closest large plane, or, with ``--sensor-in-base``, in the region predicted from
                    the rough transform and the logged pose. Reported: the method, the number of
                    mask pixels and the RMS distance of the mask's points to the fitted plane.
    border          whether the SEGMENTATION MASK (not all valid pixels: the field also holds the
                    background) comes within ``border_margin_px`` of the image border, which means
                    the board is partly out of view.
    flags           unreadable, low valid fraction, border contact, segmentation failed, plane RMS
                    above its limit, board image too small.

Logged frame. The manifest's pose is normally the board tool frame (origin at the center of the board's
front face). If the controller can only report the flange pose, log that and pass ``--target-offset-mm D``
with D the distance from the flange origin to the board's front face along the flange +z (the plate
thickness); every logged plane is then shifted by D along the logged frame's +z. Without it, such a
session shows an offset residual of about D at every pose.

Then the poses whose segmentation succeeded are registered (rigid model): per pose the normal
residual (angle between the rotated measured normal and the logged normal) and the offset residual,
each flagged above its limit. The registration drops up to ``outlier_rounds`` poses, the worst one
per round, whose residual exceeds the limits, and every pose (dropped or not) is flagged by its
residual against the solution WITHOUT the dropped poses; without this a single grossly wrong pose
would pull the solution and flag innocent poses through leverage. The
normal spread and the similarity spread of the set, with a warning below the registration defaults
(``RegistrationParameters``). A pose whose logged position was shifted shows up as an offset
residual; a wrong tool frame or a mis-typed rotation as a normal residual.

Small board image. A pose whose segmented board has fewer pixels than ``minimum_mask_pixels`` (a board at
the far standoff and a steep tilt, at the limit of the sensor's reach) gives a noisy plane, and its
residuals are poor for that reason alone, not because anything was logged wrongly. Such a pose is flagged
FLAG_SMALL_MASK instead of the two residual flags (the advice is to drop it from the plan, not to
re-capture it); it stays in the registration as before.

A set whose normal spread or similarity spread is below the registration minimum also makes the
verdict flagged (exit code 1), whatever the poses show.

Joint-sign report (``--pose-log``, optional). For sub-procedure B the pose log may carry the joint
angles at the target pose (``j1``..``j6``, degrees) and at the approach pose (``approach_j1``..
``approach_j6``). For each joint the report gives the fraction of poses whose final joint motion,
``j - approach_j``, has the majority sign: close to one means the last motion of that joint always
has the same sense, so gear backlash is taken up the same way at every pose; near one half means
the approach did not achieve that. A fraction below ``joint_sign_warn_fraction`` prints a WARNING
(informational: it does not change the exit code, because sub-procedure A has no approach to
speak of). Joint angles are taken as absolute (no wrap-around). The report is skipped, with a
note, when the columns are absent. Only the poses of the manifest are counted.

Output: a table on the console, a VERDICT line and, with ``--out``, a JSON report.

Exit code: 0 when no pose is flagged and the set is well conditioned, 1 when something is flagged,
2 when an input (manifest, pose log, rough transform) cannot be read.
"""
from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from planereg.capture.common import (EXIT_FLAGGED, EXIT_INPUT_ERROR, EXIT_OK, MATRIX_PRINT_DECIMALS, PlanInputError,
                                     board_poses_only, board_touches_border, format_number, load_rough_transform, print_error,
                                     print_warning, write_json)
from planereg.core.pipeline import DEFAULT_BORDER_MARGIN_PX, PipelineParameters, PoseMeasurement, measure_all, \
    register_measurements
from planereg.core.registration import RegistrationParameters, RegistrationResult, TransformModel, spread
from sphcal.io.capture_set import CaptureSet
from sphcal.io.poses import load_manifest

JOINT_COUNT = 6
"""Joints of the robot arm: columns j1..j6 and approach_j1..approach_j6 of the pose log."""
JOINT_COLUMN_FORMAT = "j{joint}"
APPROACH_JOINT_COLUMN_FORMAT = "approach_j{joint}"
"""Names of the pose log's joint columns, with the joint number 1..JOINT_COUNT."""
POSE_ID_COLUMN = "pose_id"
"""Column of the pose log that names the pose."""

FLAG_LOW_VALID = "low valid fraction (board flickers or is not read)"
FLAG_BORDER = "mask touches the image border (board partly out of view)"
FLAG_SEGMENTATION = "segmentation failed (board plane not found)"
FLAG_PLANE_RMS = "plane fit residual too large"
FLAG_SMALL_MASK = ("board image too small (few pixels: the pose is at the limit of the sensor's reach; "
                   "drop it from the plan rather than re-capture it)")
FLAG_NORMAL = "normal residual too large (check the logged orientation and the tool frame)"
FLAG_OFFSET = "offset residual too large (check the logged position and the tool frame)"
FLAG_NOT_REGISTERED = "not used in the registration"
REASON_REGISTRATION = "registration failed"
REASON_NORMAL_SPREAD = "normal spread below the registration minimum"
REASON_SIMILARITY_SPREAD = "similarity spread below the registration minimum"
"""Reasons (besides pose flags) that make the verdict flagged."""
ADVICE = ("re-capture the flagged poses (except those whose board image is too small: drop those from the plan), "
          "check the tool frame and the logged poses; if the spreads are low, tilt the board more and vary the "
          "standoff")
"""Advice printed with a flagged verdict."""


@dataclass(frozen=True)
class CheckParameters:
    """Thresholds of the check; the command-line defaults come from here."""

    plane_rms_warn_mm: float = 1.0
    """Flag a pose whose mask points lie farther than this (RMS) from the fitted plane."""
    min_valid_fraction: float = 0.5
    """Flag a pose whose temporal valid fraction is below this; pixels valid in fewer frames than this
    fraction are left out of the temporal-mean image."""
    border_margin_px: int = DEFAULT_BORDER_MARGIN_PX
    """Mask pixels this close to the image border mean the board is cut off."""
    minimum_mask_pixels: int = 1000
    """A board image with fewer segmented pixels than this is flagged as too small to give a reliable plane
    (and its residual flags are suppressed, since its residuals are expected to be poor). A placeholder until
    real sessions set it."""
    target_offset_mm: float = 0.0
    """Distance from the logged frame's origin to the board's front face along the logged frame's +z, in mm:
    0 when the logged pose is the board tool frame, the flange-to-board-face distance D when the controller
    can only report the flange pose (``PipelineParameters.target_offset_mm``)."""
    normal_residual_warn_deg: float = 1.0
    """Flag a pose whose normal residual after the registration exceeds this."""
    offset_residual_warn_mm: float = 1.0
    """Flag a pose whose offset residual after the registration exceeds this."""
    outlier_rounds: int = 3
    """At most this many poses are left out of the registration (the worst per round, among those whose
    residual exceeds the two limits above); 0 registers every pose, so that one wrong pose also shifts the
    residuals of the others."""
    joint_sign_warn_fraction: float = 0.9
    """Warn about a joint whose majority-sign fraction is below this."""


@dataclass
class PoseCheck:
    """Measurements and flags of one pose."""

    pose_id: str
    frames: int = 0
    valid_fraction: float = float("nan")
    method: str = ""
    pixels: int = 0
    plane_rms_mm: float = float("nan")
    touches_border: bool = False
    in_fit: bool = False                           # False when the pose was left out of the registration
    normal_residual_deg: float = float("nan")      # after the registration; NaN if the pose took no part
    offset_residual_mm: float = float("nan")
    flags: list[str] = field(default_factory=list)


@dataclass
class JointSignRow:
    """The sign statistics of one joint's final motion (target minus approach)."""

    joint: int
    positive: int
    negative: int
    zero: int
    majority_fraction: float        # NaN when no pose has a nonzero motion


@dataclass
class SetQuality:
    """Conditioning of the registered set: the spreads of the poses that took part."""

    normal_spread: float
    similarity_spread: float


# ---------------------------------------------------------------------------
# Per-pose checks
# ---------------------------------------------------------------------------
def check_pose(measurement: PoseMeasurement, params: CheckParameters) -> PoseCheck:
    """The statistics and the pose-level flags of one measured pose (the residual flags are added
    after the registration)."""
    check = PoseCheck(pose_id=measurement.pose_id, frames=measurement.frames, valid_fraction=measurement.valid_fraction,
                      touches_border=board_touches_border(measurement, params.border_margin_px))
    if measurement.segmentation is None:
        # The pipeline gives no segmentation only when the capture files could not be read; its message
        # (FLAG_UNREADABLE and the reason) says so.
        check.flags.append(measurement.error)
        return check
    check.method = measurement.segmentation.method
    check.pixels = measurement.segmentation.inlier_count
    check.plane_rms_mm = measurement.segmentation.rms_mm
    if measurement.valid_fraction < params.min_valid_fraction:
        check.flags.append(FLAG_LOW_VALID)
    if not measurement.ok:
        check.flags.append(f"{FLAG_SEGMENTATION}: {measurement.segmentation.message}")
        return check
    if check.touches_border:
        check.flags.append(FLAG_BORDER)
    if check.plane_rms_mm > params.plane_rms_warn_mm:
        check.flags.append(FLAG_PLANE_RMS)
    if check.pixels < params.minimum_mask_pixels:
        check.flags.append(FLAG_SMALL_MASK)
    return check


def registration_parameters(params: CheckParameters) -> RegistrationParameters:
    """The rigid registration of the check: the registration defaults, except that outlier rejection
    works at the check's own residual limits and for at most ``outlier_rounds`` poses. The registration's
    acceptance thresholds are not used for the verdict (the check's residual limits are); the spread
    minima gate the solve."""
    return RegistrationParameters(
        transform_model=TransformModel.RIGID, outlier_rejection_rounds=params.outlier_rounds,
        outlier_normal_residual_degrees=params.normal_residual_warn_deg,
        outlier_offset_residual_mm=params.offset_residual_warn_mm)


def apply_registration(checks: list[PoseCheck], measurements: list[PoseMeasurement], result: RegistrationResult,
                       used_ids: list[str], params: CheckParameters) -> None:
    """Fill in each registered pose's residuals and flag the ones above their limits. A pose already flagged
    FLAG_SMALL_MASK gets its residuals recorded but not flagged: they are expected to be poor, and the advice
    to check the logged pose would be wrong."""
    if not result.solved():
        return
    by_id = {check.pose_id: check for check in checks}
    for pose_id, residual in zip(used_ids, result.residuals):
        check = by_id[pose_id]
        check.in_fit = residual.used
        check.normal_residual_deg = residual.normal_angle_degrees
        check.offset_residual_mm = residual.offset_residual_mm
        if FLAG_SMALL_MASK in check.flags:
            continue
        if residual.normal_angle_degrees > params.normal_residual_warn_deg:
            check.flags.append(FLAG_NORMAL)
        if abs(residual.offset_residual_mm) > params.offset_residual_warn_mm:
            check.flags.append(FLAG_OFFSET)


def set_quality(measurements: list[PoseMeasurement]) -> SetQuality:
    """Normal spread and similarity spread of the measured poses (those with a plane), computed here
    rather than taken from the solver so that they are available even when the solver refuses a set
    whose normal spread is too low. The similarity spread is of [m_k, -d_k / max d] with m_k the logged
    board normal in the base frame and d_k the measured plane offset (the solver's similarity gate)."""
    usable = [m for m in measurements if m.ok]
    if not usable:
        return SetQuality(0, 0)
    normals = np.array([m.base_plane.normal for m in usable])
    offsets = np.array([m.sensor_plane.normalized().offset for m in usable])
    return SetQuality(spread(normals), spread(np.column_stack([normals, -offsets / np.max(np.abs(offsets))])))


# ---------------------------------------------------------------------------
# Joint-sign report
# ---------------------------------------------------------------------------
def read_joint_motions(pose_log: Path, pose_ids: set[str]) -> tuple[dict[int, list[float]], str | None]:
    """The final joint motions ``j - approach_j`` per joint (degrees) over the pose-log rows whose pose id
    is in ``pose_ids`` and whose two cells are filled in. Returns ({joint: [motions]}, note); the note
    says why the report is skipped when no joint has both columns. Raises OSError for an unreadable
    file and ValueError for a cell that is not a number."""
    with Path(pose_log).open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        columns = [name.strip() for name in (reader.fieldnames or [])]
        rows = [{(key or "").strip(): value for key, value in row.items()} for row in reader]
    joints = [joint for joint in range(1, JOINT_COUNT + 1)
              if JOINT_COLUMN_FORMAT.format(joint=joint) in columns
              and APPROACH_JOINT_COLUMN_FORMAT.format(joint=joint) in columns]
    if not joints:
        return {}, (f"the pose log {pose_log} has no joint columns ({JOINT_COLUMN_FORMAT.format(joint=1)}.."
                    f"{JOINT_COLUMN_FORMAT.format(joint=JOINT_COUNT)} together with "
                    f"{APPROACH_JOINT_COLUMN_FORMAT.format(joint=1)}.."
                    f"{APPROACH_JOINT_COLUMN_FORMAT.format(joint=JOINT_COUNT)}), so the joint-sign report is skipped")
    motions: dict[int, list[float]] = {joint: [] for joint in joints}
    for row in rows:
        if (row.get(POSE_ID_COLUMN) or "").strip() not in pose_ids:
            continue
        for joint in joints:
            target = (row.get(JOINT_COLUMN_FORMAT.format(joint=joint)) or "").strip()
            approach = (row.get(APPROACH_JOINT_COLUMN_FORMAT.format(joint=joint)) or "").strip()
            if target and approach:
                motions[joint].append(float(target) - float(approach))
    return motions, None


def joint_sign_rows(motions: dict[int, list[float]]) -> list[JointSignRow]:
    """Per joint: how many final motions are positive, negative and zero, and the fraction of the nonzero
    ones that has the majority sign."""
    rows = []
    for joint, values in sorted(motions.items()):
        values = np.asarray(values, dtype=np.float64)
        positive, negative = int((values > 0).sum()), int((values < 0).sum())
        nonzero = positive + negative
        rows.append(JointSignRow(joint, positive, negative, int(values.size - nonzero),
                                 max(positive, negative) / nonzero if nonzero else float("nan")))
    return rows


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def format_table(checks: list[PoseCheck]) -> str:
    """Console table of all poses."""
    width = max([len("pose_id")] + [len(c.pose_id) for c in checks])
    header = (f"{'pose_id':<{width}}  {'frames':>6} {'valid':>6} {'method':<20} {'pixels':>7} {'rms_mm':>7} "
              f"{'border':>6} {'in_fit':>6} {'normal_deg':>10} {'offset_mm':>10}  flags")
    lines = [header, "-" * len(header)]
    for c in checks:
        lines.append(
            f"{c.pose_id:<{width}}  {c.frames:>6d} {format_number(c.valid_fraction, '.2f'):>6} {c.method or '-':<20} "
            f"{c.pixels:>7d} {format_number(c.plane_rms_mm, '.3f'):>7} {'yes' if c.touches_border else 'no':>6} "
            f"{'yes' if c.in_fit else 'no':>6} {format_number(c.normal_residual_deg, '.3f'):>10} {format_number(c.offset_residual_mm, '+.3f'):>10}  "
            f"{'; '.join(c.flags) if c.flags else 'ok'}")
    return "\n".join(lines)


def format_joint_table(rows: list[JointSignRow], params: CheckParameters) -> str:
    """Console table of the joint-sign report."""
    lines = ["Joint-sign report (final motion = target - approach; fraction of poses with the majority sign):",
             f"  {'joint':>5} {'positive':>8} {'negative':>8} {'zero':>5} {'majority_fraction':>17}  note"]
    for row in rows:
        note = ("" if not np.isfinite(row.majority_fraction) or row.majority_fraction >= params.joint_sign_warn_fraction
                else "below the warning fraction")
        lines.append(f"  {'j' + str(row.joint):>5} {row.positive:>8d} {row.negative:>8d} {row.zero:>5d} "
                     f"{format_number(row.majority_fraction, '.3f'):>17}  {note}")
    return "\n".join(lines)


def verdict_reason(flag: str) -> str:
    """The reason a flag counts under in the verdict line: the flag without a trailing detail after a colon
    (the segmentation message), except FLAG_SMALL_MASK, whose own text contains a colon."""
    return flag if flag == FLAG_SMALL_MASK else flag.split(":")[0]


def verdict_line(checks: list[PoseCheck], extra_reasons: list[str]) -> str:
    """The final verdict: how many poses are flagged and why."""
    flagged = [c for c in checks if c.flags]
    if not flagged and not extra_reasons:
        return f"VERDICT: all {len(checks)} poses passed."
    reasons: dict[str, int] = {}
    for check in flagged:
        for flag in check.flags:
            key = verdict_reason(flag)
            reasons[key] = reasons.get(key, 0) + 1
    parts = [f"{count} x {reason}" for reason, count in reasons.items()] + extra_reasons
    return f"VERDICT: {len(flagged)} of {len(checks)} poses flagged ({'; '.join(parts)}). Advice: {ADVICE}."


def report_dictionary(checks: list[PoseCheck], result: RegistrationResult, quality: SetQuality,
                      joint_rows: list[JointSignRow] | None, params: CheckParameters, verdict: str,
                      notes: list[str], rough: bool) -> dict:
    """The JSON report."""
    return {
        "parameters": asdict(params),
        "predicted_region_used": rough,
        "poses": [asdict(c) for c in checks],
        "registration": {"status": result.status.value, "message": result.message, "poses_used": result.poses_used,
                         "rms_normal_residual_deg": result.rms_normal_residual_degrees,
                         "rms_offset_residual_mm": result.rms_offset_residual_mm,
                         "sensor_to_base": result.sensor_to_base_matrix().reshape(-1).tolist() if result.solved() else None},
        "normal_spread": quality.normal_spread,
        "similarity_spread": quality.similarity_spread,
        "joint_sign": None if joint_rows is None else [asdict(row) for row in joint_rows],
        "notes": notes,
        "n_flagged": sum(bool(c.flags) for c in checks),
        "verdict": verdict}


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    d = CheckParameters()
    parser = argparse.ArgumentParser(
        description="Quick check of a capture set before the long registration: valid pixels, border contact of the "
                    "board mask, plane fit residuals, agreement of each pose with the registration of the set, "
                    "conditioning of the set, and (with a pose log) the joint motion signs.")
    parser.add_argument("--manifest", required=True, type=Path, metavar="PATH", help="manifest CSV or JSON")
    parser.add_argument("--sensor-in-base", type=Path, metavar="PATH",
                        help="rough sensor-to-base JSON (the bootstrap tool's output); enables the predicted region")
    parser.add_argument("--pose-log", type=Path, metavar="PATH",
                        help="pose log CSV with the joint columns j1..j6 and approach_j1..approach_j6 (optional)")
    parser.add_argument("--plane-rms-warn-mm", type=float, default=d.plane_rms_warn_mm,
                        help="flag a pose whose plane fit RMS exceeds this (default %(default)s)")
    parser.add_argument("--min-valid-fraction", type=float, default=d.min_valid_fraction,
                        help="flag a pose valid in a smaller fraction of its frames (default %(default)s)")
    parser.add_argument("--border-margin-px", type=int, default=d.border_margin_px,
                        help="flag a board mask within this many pixels of the border (default %(default)s)")
    parser.add_argument("--target-offset-mm", type=float, default=d.target_offset_mm,
                        help="distance from the logged frame's origin to the board's front face along its +z, in mm; "
                             "0 when the logged frame is the board tool frame, the flange-face-to-front-face distance D "
                             "of the procedure when the flange pose was logged (default %(default)s)")
    parser.add_argument("--min-mask-pixels", type=int, default=d.minimum_mask_pixels,
                        help="flag a board image with fewer segmented pixels than this as too small for a reliable "
                             "plane; its residual flags are then suppressed (default %(default)s)")
    parser.add_argument("--normal-residual-warn-deg", type=float, default=d.normal_residual_warn_deg,
                        help="flag a pose whose normal residual exceeds this (default %(default)s)")
    parser.add_argument("--offset-residual-warn-mm", type=float, default=d.offset_residual_warn_mm,
                        help="flag a pose whose offset residual exceeds this (default %(default)s)")
    parser.add_argument("--outlier-rounds", type=int, default=d.outlier_rounds,
                        help="leave out up to this many poses (the worst per round) from the registration, so that "
                             "one wrong pose does not flag the others (default %(default)s; 0 keeps every pose)")
    parser.add_argument("--joint-sign-warn-fraction", type=float, default=d.joint_sign_warn_fraction,
                        help="warn about a joint whose majority-sign fraction is below this (default %(default)s)")
    parser.add_argument("--out", type=Path, metavar="PATH", help="write a JSON report here")
    return parser


def parameters_from_arguments(args: argparse.Namespace) -> CheckParameters:
    """The CheckParameters of the parsed command line."""
    return CheckParameters(
        plane_rms_warn_mm=args.plane_rms_warn_mm, min_valid_fraction=args.min_valid_fraction,
        border_margin_px=args.border_margin_px, minimum_mask_pixels=args.min_mask_pixels,
        target_offset_mm=args.target_offset_mm,
        normal_residual_warn_deg=args.normal_residual_warn_deg,
        offset_residual_warn_mm=args.offset_residual_warn_mm, outlier_rounds=args.outlier_rounds,
        joint_sign_warn_fraction=args.joint_sign_warn_fraction)


def main(argv: list[str] | None = None) -> int:
    """Run the tool; returns the exit code (0 nothing flagged, 1 flagged, 2 input must be fixed)."""
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
    rough = None
    if args.sensor_in_base is not None:
        try:
            rough, _source, rough_lines = load_rough_transform(args.sensor_in_base)
        except PlanInputError as error:
            print_error(str(error))
            return EXIT_INPUT_ERROR
        for line in rough_lines:
            print(line, file=sys.stderr)
    joint_rows, notes = None, []
    if args.pose_log is not None:
        try:
            motions, note = read_joint_motions(args.pose_log, set(capture_set.pose_ids()))
        except (OSError, ValueError) as error:
            print_error(f"cannot read the pose log {args.pose_log}: {error}")
            return EXIT_INPUT_ERROR
        if note is not None:
            notes.append(note)
        else:
            joint_rows = joint_sign_rows(motions)

    pipeline = PipelineParameters(min_valid_fraction=params.min_valid_fraction, border_margin_px=params.border_margin_px,
                                  target_offset_mm=params.target_offset_mm)
    measurements = measure_all(capture_set, pipeline, rough)
    checks = [check_pose(m, params) for m in measurements]
    result, used_ids = register_measurements(measurements, registration_parameters(params))
    apply_registration(checks, measurements, result, used_ids, params)
    quality = set_quality(measurements)
    extra_reasons: list[str] = []
    if not result.solved():
        extra_reasons.append(f"{REASON_REGISTRATION}: {result.message}")
    registration_defaults = RegistrationParameters()
    if quality.normal_spread < registration_defaults.minimum_normal_spread:
        extra_reasons.append(REASON_NORMAL_SPREAD)
    if result.solved() and quality.similarity_spread < registration_defaults.minimum_similarity_spread:
        extra_reasons.append(REASON_SIMILARITY_SPREAD)

    print(format_table(checks))
    for check in checks:
        for flag in check.flags:
            print_warning(f"pose {check.pose_id}: {flag}.")
    for note in notes:
        print(f"NOTE: {note}")
    if result.solved():
        print("Sensor-to-base transform of this set (row-major 4x4):")
        print(np.array2string(result.sensor_to_base_matrix(), precision=MATRIX_PRINT_DECIMALS, suppress_small=True))
    print(f"Registration status: {result.status.value}: {result.message}")
    print(f"Normal spread {quality.normal_spread:.4f} (minimum {registration_defaults.minimum_normal_spread:g}); "
          f"similarity spread {quality.similarity_spread:.4f} (minimum {registration_defaults.minimum_similarity_spread:g})")
    for reason in (REASON_NORMAL_SPREAD, REASON_SIMILARITY_SPREAD):
        if reason in extra_reasons:
            print_warning(f"the set of {args.manifest}: {reason}; tilt the board more and vary the standoff.")
    if joint_rows is not None:
        print(format_joint_table(joint_rows, params))
        for row in joint_rows:
            if np.isfinite(row.majority_fraction) and row.majority_fraction < params.joint_sign_warn_fraction:
                print_warning(f"pose log {args.pose_log}: joint j{row.joint} moves in the same direction in only "
                              f"{row.majority_fraction:.2f} of the final approach motions; backlash is not taken up "
                              "the same way at every pose for this joint.")
    verdict = verdict_line(checks, extra_reasons)
    print(verdict)
    if args.out is not None:
        write_json(args.out, report_dictionary(checks, result, quality, joint_rows, params, verdict, notes,
                                               rough is not None))
    return EXIT_FLAGGED if (any(c.flags for c in checks) or extra_reasons) else EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
