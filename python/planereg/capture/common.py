"""
Helpers shared by the three capture tools (plan_poses, bootstrap, check_captures): exit codes, the
message format, reading the rough sensor-to-base transform, restricting a capture set to board poses,
and JSON output that survives NaN.

Message format (code design, Section 7): every message to the technician starts with ``ERROR:`` or
``WARNING:`` and names the pose or the file it is about.

Units: millimeters and degrees at every interface.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

from planereg.core.pipeline import PoseMeasurement, mask_touches_border
from sphcal.cli.plan_poses import PlanInputError, load_sensor_in_base
from sphcal.geometry.transforms import RigidTransform
from sphcal.io.capture_set import CaptureSet
from sphcal.io.poses import TARGET_KIND_BOARD

EXIT_OK = 0
"""Exit code: the tool ran and nothing was flagged."""
EXIT_FLAGGED = 1
"""Exit code: the tool ran and flagged something the technician has to look at."""
EXIT_INPUT_ERROR = 2
"""Exit code: an input (manifest, file, option) must be fixed before the tool can run."""

ERROR_PREFIX = "ERROR:"
WARNING_PREFIX = "WARNING:"
"""Every message to the technician starts with one of these."""

REPORT_JSON_INDENT = 2
"""Indentation of the JSON files the tools write."""
MATRIX_PRINT_DECIMALS = 3
"""Decimals printed for a transform matrix."""
UNAVAILABLE_TEXT = "-"
"""Printed in a table cell whose number is NaN or infinite."""

ROUGH_TRANSFORM_RESIDUAL_WARN_MM = 5.0
"""A bootstrap given as sensor/base point pairs (sphcal's other --sensor-in-base format) is
warned about when a pair disagrees with the solved transform by more than this."""
STATUS_SUCCESS_TEXT = "Success"
"""Value of the ``status`` key of a bootstrap JSON whose registration met every threshold."""


def error_message(text: str) -> str:
    """``ERROR: <text>``."""
    return f"{ERROR_PREFIX} {text}"


def warning_message(text: str) -> str:
    """``WARNING: <text>``."""
    return f"{WARNING_PREFIX} {text}"


def print_error(text: str) -> None:
    """Print an error line to standard error."""
    print(error_message(text), file=sys.stderr)


def print_warning(text: str) -> None:
    """Print a warning line to standard error."""
    print(warning_message(text), file=sys.stderr)


def format_number(value: float, spec: str) -> str:
    """``format(value, spec)``, or a dash for NaN and infinity (a table cell without a value)."""
    return UNAVAILABLE_TEXT if not np.isfinite(value) else format(value, spec)


def json_safe(value):
    """Convert numpy values to plain JSON types; NaN and infinity become null (JSON has neither)."""
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, (np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def write_json(path: Path, document: dict) -> None:
    """Write a JSON document (NaN as null), creating the parent directory."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(json_safe(document), indent=REPORT_JSON_INDENT), encoding="utf-8")


def load_rough_transform(path: Path) -> tuple[RigidTransform, str, list[str]]:
    """Read a ``--sensor-in-base`` file with sphcal's reader: either ``{"matrix": [16 numbers]}`` (the
    bootstrap tool writes this) or sphcal's point-pair ``{"bootstrap": [...]}``. Returns the
    sensor-to-base transform, a description of its source and report lines. Raises PlanInputError,
    whose message says what to fix, for an unreadable or invalid file.

    A file written by the bootstrap tool also carries the registration ``status``; when it is not
    Success the transform is probably not good enough to plan or segment with, and a WARNING line
    saying so is appended to the report lines."""
    transform, source, lines = load_sensor_in_base(path, ROUGH_TRANSFORM_RESIDUAL_WARN_MM)
    lines = list(lines)
    document = json.loads(Path(path).read_text(encoding="utf-8"))   # load_sensor_in_base proved it is valid JSON
    status = document.get("status") if isinstance(document, dict) else None
    if status is not None and status != STATUS_SUCCESS_TEXT:
        lines.append(warning_message(
            f"{path} was written by a bootstrap run whose registration status is {status}, not "
            f"{STATUS_SUCCESS_TEXT}; its transform may be too far off. Re-run the bootstrap with better poses."))
    return transform, source, lines


def board_touches_border(measurement: PoseMeasurement, margin_px: int) -> bool:
    """True when the board's segmentation mask, taken BEFORE the segmentation's erosion (the board's
    full extent; the erosion treats the image border as background and would pull a cut-off board
    inside the margin), comes within ``margin_px`` of the image border: the board is partly out of
    view. The pipeline computes this with the margin it was given; here the margin is re-applied so
    that the tools' own ``--border-margin-px`` is honored."""
    if measurement.segmentation is None or not measurement.ok:
        return False
    segmentation = measurement.segmentation
    full_extent = segmentation.unshrunk_mask if segmentation.unshrunk_mask is not None else segmentation.mask
    return mask_touches_border(full_extent, margin_px)


def board_poses_only(capture_set: CaptureSet) -> tuple[CaptureSet, list[str]]:
    """The capture set restricted to board poses, and one WARNING per pose that was left out because
    its target is not a board (the registration uses boards only)."""
    kept = [record for record in capture_set.records if record.target_kind == TARGET_KIND_BOARD]
    skipped_ids = [pose_id for pose_id in capture_set.pose_ids()
                   if capture_set.records_for(pose_id)[0].target_kind != TARGET_KIND_BOARD]
    warnings = [warning_message(f"pose {pose_id}: its target is not a board, so it is left out; "
                                "the registration uses board captures only") for pose_id in skipped_ids]
    return CaptureSet(kept), warnings


# PlanInputError is sphcal's exception for an input the technician must fix; it is re-exported here so that
# the tools raise and catch one class.
__all__ = ["PlanInputError", "EXIT_OK", "EXIT_FLAGGED", "EXIT_INPUT_ERROR"]
