"""
Target-plane segmentation: find the board's front face in a depth capture and fit its plane.

Two ways to find the candidate region, then one refinement:

Predicted region (when a rough sensor-to-base transform is known). The board's reported pose
and the rough transform predict where the board lies in the sensor frame; the candidate pixels
are those whose points lie within ``prediction_band_mm`` of the predicted plane and inside the
predicted rectangle shrunk by ``prediction_edge_margin_mm``.

Closest large plane (no prediction, e.g. the bootstrap captures). Sequential RANSAC over the
valid points extracts up to ``max_candidate_planes`` planes; for each, the largest
8-connected image component of its inliers is measured in physical area (sum of the pixel
footprints, z^2 / (fx fy) / cos(incidence)); components smaller than ``min_plane_area_mm2`` or
not facing the sensor are discarded, and of the rest the one with the smallest median depth is
the target. A table or wall behind the board is larger but farther; a hand or a cable is
closer but small.

Refinement (both). The plane is fitted to the candidate points by principal components; points
farther from it than ``flyaway_sigma_multiple`` robust scales are dropped; the mask is reduced
to its largest connected component and eroded by ``edge_shrink_px`` (fly-away pixels sit on the
silhouette boundary where a pixel mixes the board and the background); the fit is repeated
until the plane stops changing or the rounds run out. The total erosion is capped so the
region does not vanish on a small or distant board.

Inputs are the temporal-mean points of a pose (H, W, 3) with NaN where invalid, as
``sphcal.features.depth_features.temporal_mean_points`` produces them. Image arrays are
(height, width); pixel (u, v) = (column, row). Units: millimeters.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from planereg.core.planes import Plane, predicted_sensor_plane
from sphcal.geometry.camera import PinholeCamera
from sphcal.geometry.transforms import RigidTransform

MEDIAN_ABSOLUTE_TO_SIGMA = 1.4826
"""Factor converting a median absolute residual into a Gaussian sigma."""
POINTS_PER_PLANE_SAMPLE = 3
"""RANSAC draws three points to define a plane."""
MIN_SAMPLE_TRIANGLE_AREA_MM2 = 25.0
"""A RANSAC sample whose three points span less than this area (|cross| / 2) is skipped as degenerate."""
EIGHT_CONNECTED = np.ones((3, 3), dtype=bool)
"""Structuring element for image components and erosion: 8-connectivity / square erosion."""
MIN_FOOTPRINT_COSINE = 0.05
"""Floor on cos(incidence) in the pixel footprint, so grazing pixels do not get absurd areas."""

METHOD_PREDICTED = "predicted_region"
METHOD_CLOSEST = "closest_large_plane"


@dataclass(frozen=True)
class SegmentationParameters:
    """All thresholds of the segmentation; the defaults suit a 200 x 150 mm board between 0.3 and
    1.2 m from a 640 x 480 sensor and are to be revised on real captures."""

    min_plane_area_mm2: float = 100.0 * 100.0
    """A plane counts as "large" when its largest connected component covers at least this area."""
    ransac_distance_mm: float = 3.0
    """Inlier distance of the RANSAC plane search."""
    ransac_iterations: int = 300
    """Random three-point samples per extracted plane."""
    ransac_sample_pixels: int = 20000
    """At most this many valid pixels take part in the RANSAC search (a random subset)."""
    max_candidate_planes: int = 6
    """Planes extracted sequentially before giving up."""
    min_facing_cosine: float = 0.25
    """A candidate must face the sensor: cos of the angle between its normal (toward the sensor)
    and the direction from its centroid to the sensor must exceed this (0.25 = 75 degrees)."""
    prediction_band_mm: float = 25.0
    """Points within this distance of the predicted plane are candidates."""
    prediction_edge_margin_mm: float = 10.0
    """The predicted rectangle is shrunk by this on every side."""
    edge_shrink_px: int = 2
    """Erosion of the mask per refinement round."""
    max_total_edge_shrink_px: int = 10
    """Cap on the accumulated erosion."""
    flyaway_sigma_multiple: float = 4.0
    """Points beyond this many robust scales from the plane are dropped each round."""
    closing_px: int = 1
    """Morphological closing applied to the inlier mask before its largest connected component is
    taken, so that noise-clipped pixels do not cut the board into fragments."""
    min_robust_scale_mm: float = 0.05
    """Floor on the robust scale, so a near-perfect fit does not reject everything."""
    max_refit_rounds: int = 6
    convergence_angle_degrees: float = 0.005
    """Refinement stops when the normal moved less than this ..."""
    convergence_offset_mm: float = 0.005
    """... and the offset moved less than this between rounds."""
    min_inlier_pixels: int = 300
    """Fewer pixels than this is a failed segmentation."""
    seed: int = 0
    """Seed of the RANSAC random draws (reproducible)."""


@dataclass
class PlanePrediction:
    """Where the board is expected in the sensor frame."""

    plane_sensor: Plane                 # predicted plane, unit normal toward the sensor
    board_to_sensor: RigidTransform     # predicted board tool frame in the sensor frame
    half_size_mm: tuple[float, float]   # (half width along board x, half height along board y)


@dataclass
class SegmentationResult:
    ok: bool
    method: str                         # METHOD_PREDICTED or METHOD_CLOSEST
    message: str
    mask: np.ndarray                    # (H, W) bool, the pixels the final plane was fitted to
    plane: Plane | None                 # unit normal toward the sensor, offset > 0
    rms_mm: float                       # RMS distance of the mask's points to the plane
    inlier_count: int
    rounds: int                         # refinement rounds run
    edge_shrink_px: int                 # total erosion applied
    candidate_count: int = 0            # planes found by the closest-large-plane search
    candidate_areas_mm2: list = None    # their areas, for the report  # type: ignore[assignment]
    unshrunk_mask: np.ndarray | None = None  # the inliers of the first round BEFORE any erosion: the
                                        # board's full extent, used for the image-border test

    def __post_init__(self) -> None:
        if self.candidate_areas_mm2 is None:
            self.candidate_areas_mm2 = []


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def valid_mask(points: np.ndarray) -> np.ndarray:
    """Pixels with a finite point in front of the sensor (z > 0)."""
    return np.isfinite(points).all(axis=-1) & (points[..., 2] > 0.0)


def fit_plane_pca(points: np.ndarray) -> Plane:
    """Total least-squares plane through (N, 3) points: the normal is the singular vector of the
    smallest singular value of the centered points. Oriented toward the sensor (origin on the
    positive side, offset > 0)."""
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vt[-1]
    return Plane(normal, -float(normal @ centroid)).normalized(origin_on_positive_side=True)


def pixel_footprint_mm2(points: np.ndarray, camera: PinholeCamera, normal: np.ndarray) -> np.ndarray:
    """Physical area covered by each pixel of a surface with the given unit normal:
    z^2 / (fx fy) / |cos(incidence)|, (H, W), NaN where the point is invalid."""
    z = points[..., 2]
    rays = camera.ray_directions()
    cosine = np.maximum(np.abs(rays @ normal), MIN_FOOTPRINT_COSINE)
    return z * z / (camera.focal_x_px * camera.focal_y_px) / cosine


def largest_component(mask: np.ndarray) -> np.ndarray:
    """The largest 8-connected True region of the mask (all False if the mask is empty)."""
    labels, count = ndimage.label(mask, structure=EIGHT_CONNECTED)
    if count == 0:
        return np.zeros_like(mask)
    sizes = ndimage.sum(mask, labels, index=np.arange(1, count + 1))
    return labels == (int(np.argmax(sizes)) + 1)


def board_prediction(sensor_to_base_rough: RigidTransform, board_to_base: RigidTransform,
                     half_size_mm: tuple[float, float], target_offset_mm: float = 0.0,
                     scale: float = 1.0) -> PlanePrediction:
    """Predict the board in the sensor frame from a rough sensor-to-base transform and the
    reported board pose (REVIEW.md Section 5.5: the predictive model of the segmentation).

    ``scale`` is the similarity model's c in X = [c R, s; 0 1] (1 for a rigid transform): a point
    p_b of the base frame is seen by the sensor at p_s = R^t (p_b - s) / c, so the predicted board
    frame's translation and the board's half sizes are divided by c as well."""
    from planereg.core.planes import board_plane_in_base  # local import keeps module order simple
    base_plane = board_plane_in_base(board_to_base, target_offset_mm)
    plane_sensor = predicted_sensor_plane(sensor_to_base_rough, base_plane, scale).normalized(origin_on_positive_side=True)
    rigid_board_to_sensor = sensor_to_base_rough.inverse().compose(board_to_base)
    board_to_sensor = RigidTransform(rigid_board_to_sensor.rotation, rigid_board_to_sensor.translation / float(scale))
    return PlanePrediction(plane_sensor, board_to_sensor,
                           (float(half_size_mm[0]) / float(scale), float(half_size_mm[1]) / float(scale)))


# ---------------------------------------------------------------------------
# Candidate regions
# ---------------------------------------------------------------------------
def predicted_region(points: np.ndarray, valid: np.ndarray, prediction: PlanePrediction,
                     params: SegmentationParameters) -> np.ndarray:
    """Pixels whose points lie within the band of the predicted plane and inside the predicted
    rectangle shrunk by the edge margin (coordinates taken in the predicted board frame)."""
    rotation, translation = prediction.board_to_sensor.rotation, prediction.board_to_sensor.translation
    local = (np.where(valid[..., None], points, 0.0) - translation) @ rotation   # R^t (p - t)
    half_width = prediction.half_size_mm[0] - params.prediction_edge_margin_mm
    half_height = prediction.half_size_mm[1] - params.prediction_edge_margin_mm
    inside = ((np.abs(local[..., 0]) <= half_width) & (np.abs(local[..., 1]) <= half_height)
              & (np.abs(local[..., 2]) <= params.prediction_band_mm))
    return valid & inside


def _ransac_plane(sample_points: np.ndarray, params: SegmentationParameters, rng: np.random.Generator):
    """Best plane among random three-point samples by inlier count; (plane, inlier mask over the
    sample) or (None, None) when no sample was usable."""
    count = sample_points.shape[0]
    if count < POINTS_PER_PLANE_SAMPLE:
        return None, None
    best_plane, best_inliers, best_count = None, None, -1
    for _ in range(params.ransac_iterations):
        a, b, c = sample_points[rng.choice(count, size=POINTS_PER_PLANE_SAMPLE, replace=False)]
        cross = np.cross(b - a, c - a)
        area = 0.5 * float(np.linalg.norm(cross))
        if area < MIN_SAMPLE_TRIANGLE_AREA_MM2:
            continue
        normal = cross / (2.0 * area)
        distance = np.abs((sample_points - a) @ normal)
        inliers = distance <= params.ransac_distance_mm
        inlier_count = int(inliers.sum())
        if inlier_count > best_count:
            best_plane, best_inliers, best_count = Plane(normal, -float(normal @ a)), inliers, inlier_count
    return best_plane, best_inliers


def closest_large_plane(points: np.ndarray, valid: np.ndarray, camera: PinholeCamera,
                        params: SegmentationParameters) -> tuple[np.ndarray | None, list[float], str]:
    """The candidate region of the closest large plane (module docstring). Returns (mask or None,
    the areas of every accepted candidate, a message)."""
    rng = np.random.default_rng(params.seed)
    remaining = valid.copy()
    accepted: list[tuple[float, np.ndarray, float]] = []   # (median depth, mask, area)
    for _ in range(params.max_candidate_planes):
        remaining_indices = np.flatnonzero(remaining)
        if remaining_indices.size < params.min_inlier_pixels:
            break
        if remaining_indices.size > params.ransac_sample_pixels:
            remaining_indices = rng.choice(remaining_indices, size=params.ransac_sample_pixels, replace=False)
        sample = points.reshape(-1, 3)[remaining_indices]
        plane, _ = _ransac_plane(sample, params, rng)
        if plane is None:
            break
        plane = plane.normalized(origin_on_positive_side=True)
        distance = np.abs(np.where(valid, plane.signed_distance(points), np.inf))
        inliers = remaining & (distance <= params.ransac_distance_mm)
        component = largest_component(inliers)
        remaining &= ~inliers                      # never find the same plane again
        if component.sum() < params.min_inlier_pixels:
            continue
        area = float(np.nansum(pixel_footprint_mm2(points, camera, plane.normal)[component]))
        centroid = points[component].mean(axis=0)
        facing = float(plane.normal @ (-centroid / np.linalg.norm(centroid)))
        if area < params.min_plane_area_mm2 or facing < params.min_facing_cosine:
            continue
        accepted.append((float(np.median(points[component][:, 2])), component, area))
    if not accepted:
        return None, [], "no plane of sufficient area facing the sensor was found"
    accepted.sort(key=lambda item: item[0])
    return accepted[0][1], [item[2] for item in accepted], f"{len(accepted)} large plane(s); the closest was taken"


# ---------------------------------------------------------------------------
# Refinement
# ---------------------------------------------------------------------------
def refine_plane(points: np.ndarray, mask: np.ndarray, params: SegmentationParameters):
    """Iterated fit / fly-away rejection / component selection / erosion (module docstring).
    Returns (plane or None, final mask, rms, rounds, total erosion, message, unshrunk mask)."""
    previous: Plane | None = None
    plane: Plane | None = None
    unshrunk: np.ndarray | None = None
    total_shrink, rounds = 0, 0
    for rounds in range(1, params.max_refit_rounds + 1):
        if int(mask.sum()) < params.min_inlier_pixels:
            return None, mask, float("nan"), rounds, total_shrink, (
                f"only {int(mask.sum())} candidate pixels, fewer than the minimum {params.min_inlier_pixels}"), unshrunk
        plane = fit_plane_pca(points[mask])
        distance = np.where(mask, plane.signed_distance(np.where(mask[..., None], points, 0.0)), np.nan)
        scale = max(MEDIAN_ABSOLUTE_TO_SIGMA * float(np.nanmedian(np.abs(distance[mask]))), params.min_robust_scale_mm)
        inliers = mask & (np.abs(np.nan_to_num(distance, nan=np.inf)) <= params.flyaway_sigma_multiple * scale)
        # The sensor's block-correlated noise can cut a far, tilted board into fragments by the clip
        # above; a closing bridges one-pixel gaps before the largest component is taken, and the
        # intersection keeps the clipped pixels themselves out of the fit.
        inliers = largest_component(ndimage.binary_closing(inliers, structure=EIGHT_CONNECTED,
                                                           iterations=params.closing_px)) & inliers
        if unshrunk is None:
            unshrunk = inliers.copy()      # full extent of the board, before any erosion
        if total_shrink < params.max_total_edge_shrink_px and params.edge_shrink_px > 0:
            eroded = ndimage.binary_erosion(inliers, structure=EIGHT_CONNECTED, iterations=params.edge_shrink_px)
            # Erode only while the region stays usable: a small or distant board, or one partly
            # outside the field, keeps its un-eroded inliers rather than vanishing.
            if int(eroded.sum()) >= params.min_inlier_pixels:
                inliers = eroded
                total_shrink += params.edge_shrink_px
        converged = (previous is not None
                     and plane.angle_to_degrees(previous) < params.convergence_angle_degrees
                     and abs(plane.offset - previous.offset) < params.convergence_offset_mm)
        previous, mask = plane, inliers
        if converged:
            break
    if int(mask.sum()) < params.min_inlier_pixels or plane is None:
        return None, mask, float("nan"), rounds, total_shrink, "the region shrank below the minimum pixel count", unshrunk
    plane = fit_plane_pca(points[mask])
    rms = float(np.sqrt(np.mean(plane.signed_distance(points[mask]) ** 2)))
    return plane, mask, rms, rounds, total_shrink, ("converged" if previous is not None else "rounds exhausted"), unshrunk


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def segment_target_plane(points: np.ndarray, camera: PinholeCamera,
                         params: SegmentationParameters = SegmentationParameters(),
                         prediction: PlanePrediction | None = None) -> SegmentationResult:
    """Find the target plane in the temporal-mean points (H, W, 3) of one pose.

    With ``prediction`` the candidate region is the predicted one; if it yields too few pixels
    (a wrong rough transform, or the board outside the field), the closest-large-plane search is
    used instead and the result's ``method`` says so. The returned plane has a unit normal toward
    the sensor and a positive offset.
    """
    valid = valid_mask(points)
    candidate_areas: list[float] = []
    method, message = METHOD_CLOSEST, ""
    mask: np.ndarray | None = None
    if prediction is not None:
        region = predicted_region(points, valid, prediction, params)
        if int(region.sum()) >= params.min_inlier_pixels:
            mask, method, message = region, METHOD_PREDICTED, "predicted region"
        else:
            message = (f"predicted region holds only {int(region.sum())} pixels; "
                       "fell back to the closest large plane. ")
    if mask is None:
        mask, candidate_areas, search_message = closest_large_plane(points, valid, camera, params)
        message += search_message
        if mask is None:
            return SegmentationResult(ok=False, method=METHOD_CLOSEST, message=message, mask=np.zeros_like(valid),
                                      plane=None, rms_mm=float("nan"), inlier_count=0, rounds=0, edge_shrink_px=0,
                                      candidate_count=0, candidate_areas_mm2=candidate_areas)
    plane, final_mask, rms, rounds, shrink, refine_message, unshrunk = refine_plane(points, mask, params)
    ok = plane is not None
    return SegmentationResult(ok=ok, method=method, message=f"{message}; {refine_message}", mask=final_mask,
                              plane=plane, rms_mm=rms, inlier_count=int(final_mask.sum()), rounds=rounds,
                              edge_shrink_px=shrink, candidate_count=len(candidate_areas),
                              candidate_areas_mm2=candidate_areas, unshrunk_mask=unshrunk)
