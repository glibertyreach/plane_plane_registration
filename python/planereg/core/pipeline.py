"""
The per-pose measurement pipeline shared by the capture check and the analysis, so that
both segment and register identically (code design, Section 7.0):

    CaptureSet.load_stack(pose_id)
      -> temporal_mean_points(stack.xyz, stack.valid, min_valid_fraction)
      -> segment_target_plane(points, camera, params, prediction)
      -> board_plane_in_base(record.target_pose_positioner, target_offset_mm)
      -> register_planes over the poses whose segmentation succeeded

With a rough sensor-to-base transform the segmentation is given a prediction of the
board's location (predicted region); without one it searches for the closest large plane.

Units: millimeters, degrees; image arrays (height, width); pixel (u, v) = (column, row).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from planereg.core.planes import Plane, board_plane_in_base
from planereg.core.registration import RegistrationParameters, RegistrationResult, RegistrationStatus, \
    register_planes
from planereg.core.segmentation import PlanePrediction, SegmentationParameters, SegmentationResult, \
    board_prediction, segment_target_plane
from sphcal.features.depth_features import temporal_mean_points
from sphcal.geometry.transforms import RigidTransform
from sphcal.io.capture_set import CaptureSet

DEFAULT_BORDER_MARGIN_PX = 4
"""Mask pixels this close to the image border mean the board is partly out of view."""


@dataclass(frozen=True)
class PipelineParameters:
    min_valid_fraction: float = 0.5
    """A pixel enters the temporal mean only if valid in at least this fraction of the frames."""
    segmentation: SegmentationParameters = field(default_factory=SegmentationParameters)
    target_offset_mm: float = 0.0
    """Offset of the board face along the logged frame's +z (0 for the board tool frame)."""
    border_margin_px: int = DEFAULT_BORDER_MARGIN_PX


@dataclass
class PoseMeasurement:
    """Everything measured about one pose."""

    pose_id: str
    kind: str
    frames: int
    valid_fraction: float                  # temporal read rate over pixels valid in any frame
    segmentation: SegmentationResult | None
    sensor_plane: Plane | None             # the segmented plane, unit normal toward the sensor
    base_plane: Plane                      # the plate plane in the base frame from the logged pose
    board_to_base: RigidTransform          # the logged pose
    half_size_mm: tuple[float, float]
    board_center_uv: tuple[float, float]   # centroid pixel of the mask (NaN if none)
    board_center_sensor_mm: np.ndarray     # mean of the mask's points (NaN if none)
    touches_border: bool
    error: str = ""                        # why the pose could not be measured, if it could not

    @property
    def ok(self) -> bool:
        return self.sensor_plane is not None


def temporal_valid_fraction(valid_stack: np.ndarray) -> float:
    """Mean, over pixels valid in at least one frame, of the fraction of frames in which they
    are valid (the same statistic as sphcal's check tool)."""
    per_pixel = valid_stack.mean(axis=0)
    seen = per_pixel > 0.0
    return float(per_pixel[seen].mean()) if seen.any() else 0.0


def mask_touches_border(mask: np.ndarray, margin_px: int) -> bool:
    """True when any True pixel of the (H, W) mask lies within margin_px of the image border."""
    if margin_px <= 0 or not mask.any():
        return False
    inner = mask[margin_px:mask.shape[0] - margin_px, margin_px:mask.shape[1] - margin_px]
    return bool(mask.sum() > inner.sum())


def measure_pose(capture_set: CaptureSet, pose_id: str, params: PipelineParameters,
                 rough_sensor_to_base: RigidTransform | None = None, rough_scale: float = 1.0) -> PoseMeasurement:
    """Load, average, segment and describe one pose. Never raises for a bad capture: the
    result's ``error`` says what went wrong and ``ok`` is False. ``rough_scale`` is the similarity
    scale that goes with ``rough_sensor_to_base`` (1 for a rigid transform)."""
    records = capture_set.records_for(pose_id)
    record = records[0]
    half_size = record.board_half_size_mm or (float("nan"), float("nan"))
    base_plane = board_plane_in_base(record.target_pose_positioner, params.target_offset_mm)
    nan3 = np.full(3, np.nan)
    try:
        stack = capture_set.load_stack(pose_id)
    except (OSError, ValueError, KeyError) as error:
        return PoseMeasurement(pose_id, record.target_kind, len(records), float("nan"), None, None, base_plane,
                               record.target_pose_positioner, half_size, (np.nan, np.nan), nan3, False,
                               f"capture files could not be read: {error}")
    points = temporal_mean_points(stack.xyz, stack.valid, params.min_valid_fraction).astype(np.float64)
    prediction: PlanePrediction | None = None
    if rough_sensor_to_base is not None and record.board_half_size_mm is not None:
        prediction = board_prediction(rough_sensor_to_base, record.target_pose_positioner,
                                      record.board_half_size_mm, params.target_offset_mm, rough_scale)
    segmentation = segment_target_plane(points, stack.camera, params.segmentation, prediction)
    if segmentation.ok:
        rows, cols = np.nonzero(segmentation.mask)
        center_uv = (float(cols.mean()), float(rows.mean()))
        center_sensor = points[segmentation.mask].mean(axis=0)
    else:
        center_uv, center_sensor = (np.nan, np.nan), nan3
    return PoseMeasurement(
        pose_id, record.target_kind, stack.xyz.shape[0], temporal_valid_fraction(stack.valid), segmentation,
        segmentation.plane if segmentation.ok else None, base_plane, record.target_pose_positioner, half_size,
        center_uv, center_sensor,
        # The border test uses the board's full extent (before erosion): the erosion treats the
        # image border as background and would pull a cut-off board inside the margin.
        mask_touches_border(segmentation.unshrunk_mask if segmentation.unshrunk_mask is not None
                            else segmentation.mask, params.border_margin_px),
        "" if segmentation.ok else f"segmentation failed: {segmentation.message}")


def measure_all(capture_set: CaptureSet, params: PipelineParameters,
                rough_sensor_to_base: RigidTransform | None = None, rough_scale: float = 1.0) -> list[PoseMeasurement]:
    """measure_pose for every pose id of the set, in order of first appearance. ``rough_scale`` is the
    similarity scale of the rough transform (1 for a rigid one); it makes the prediction exact."""
    return [measure_pose(capture_set, pose_id, params, rough_sensor_to_base, rough_scale)
            for pose_id in capture_set.pose_ids()]


def register_measurements(measurements: list[PoseMeasurement],
                          params: RegistrationParameters = RegistrationParameters()) -> tuple[RegistrationResult, list[str]]:
    """Register the poses whose segmentation succeeded. Returns the result and the list of pose
    ids that took part, in the order of the result's residuals."""
    usable = [m for m in measurements if m.ok]
    if not usable:
        return RegistrationResult(RegistrationStatus.TOO_FEW_POSES, "no pose was segmented successfully"), []
    result = register_planes([m.sensor_plane for m in usable], [m.base_plane for m in usable], params)
    return result, [m.pose_id for m in usable]
