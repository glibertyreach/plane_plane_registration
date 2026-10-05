"""
The registration solver: sensor-to-base transform from plane-plane correspondences.

This is the Python reference of the C++ library in include/lri/registration (same algorithm,
same parameter names, same statuses), following REVIEW.md Section 6:

    1. normalize every sensor plane to a unit normal pointing toward the sensor (d > 0)
    2. the base-frame plate plane of every pose, (m_k, e_k), from the reported pose
    3. conditioning gate: the "normal spread" of the m_k
    4. rotation R from the normals alone: orthogonal Procrustes, no centroid subtraction
    5. translation s (and, for the Similarity model, the scale c) from the offsets:
           s . m_k - c d_k = -e_k        (linear least squares, independent of R)
    6. X = [c R, s; 0 1]
    7. residuals per pose; optional drop-worst outlier rejection; acceptance thresholds

Input planes carry their own frames: ``sensor_planes[k]`` is the plane measured in the sensor
frame at pose k, ``base_planes[k]`` the plate plane in the base frame computed from the robot's
reported pose with :func:`planereg.core.planes.board_plane_in_base`.

Units: millimeters, degrees at the interface.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from planereg.core.planes import Plane
from sphcal.geometry.transforms import RigidTransform

RIGID_MINIMUM_POSE_COUNT = 3
"""Three poses with linearly independent normals determine a rigid transform (REVIEW.md 4.6)."""
SIMILARITY_MINIMUM_POSE_COUNT = 4
"""The scale adds one unknown to the translation system, so one more pose (REVIEW.md 4.7)."""
MAXIMUM_SPREAD_OF_UNIT_NORMALS = 1.0 / np.sqrt(3.0)
"""The normal spread of normals spread uniformly over all directions (for reference only)."""
OUTLIER_RATIO_LIMIT = 1.0
"""A pose is an outlier candidate only when a residual exceeds its threshold (ratio > 1)."""


class TransformModel(Enum):
    RIGID = "rigid"
    SIMILARITY = "similarity"


class RegistrationStatus(Enum):
    SUCCESS = "Success"
    INVALID_INPUT = "InvalidInput"
    TOO_FEW_POSES = "TooFewPoses"
    DEGENERATE_POSES = "DegeneratePoses"
    RESIDUALS_EXCEED_THRESHOLD = "ResidualsExceedThreshold"


@dataclass(frozen=True)
class RegistrationParameters:
    """Mirror of the C++ RegistrationParameters; the defaults are placeholders from the review's
    synthetic noise model (0.2 degree normals, 0.2 mm offsets), to be revised on real data."""

    transform_model: TransformModel = TransformModel.RIGID
    minimum_pose_count: int = RIGID_MINIMUM_POSE_COUNT
    """Fewest poses accepted; can only raise the model's geometric floor (3 rigid, 4 similarity)."""
    minimum_normal_spread: float = 0.05
    """Gate on the smallest singular value of the N x 3 unit-normal matrix divided by sqrt(N): the RMS
    component of the normals along their least-covered direction (0 when all plates are parallel,
    at most 0.577). About 0.04 for 17 poses tilted within +/-5 degrees, 0.16 for +/-20 degrees."""
    minimum_similarity_spread: float = 0.01
    """Same measure for the N x 4 matrix [m_k, -d_k / max|d_k|] of the similarity model; small unless
    the standoff varies across poses, because that is what makes the scale observable."""
    maximum_rms_normal_residual_degrees: float = 0.5
    maximum_rms_offset_residual_mm: float = 0.5
    maximum_normal_residual_degrees: float = 1.0
    """Worst single pose."""
    maximum_offset_residual_mm: float = 1.0
    """Worst single pose."""
    outlier_rejection_rounds: int = 0
    """At most this many poses are dropped, one per round (the worst relative to its threshold);
    0 disables rejection. One per round because a gross outlier lifts good poses over the
    threshold through leverage."""
    outlier_normal_residual_degrees: float = 1.0
    outlier_offset_residual_mm: float = 1.0


@dataclass
class PoseResidual:
    normal_angle_degrees: float   # angle between R n_k and m_k
    offset_residual_mm: float     # s . m_k + e_k - c d_k, in the base frame's length unit
    used: bool                    # False once outlier rejection dropped the pose


@dataclass
class RegistrationResult:
    status: RegistrationStatus
    message: str
    sensor_to_base: RigidTransform = field(default_factory=RigidTransform.identity)  # rotation R and translation s
    scale: float = 1.0                                                              # c; exactly 1 for the rigid model
    residuals: list[PoseResidual] = field(default_factory=list)
    poses_used: int = 0
    normal_spread: float = 0.0
    similarity_spread: float = 0.0
    rms_normal_residual_degrees: float = float("nan")
    max_normal_residual_degrees: float = float("nan")
    rms_offset_residual_mm: float = float("nan")
    max_offset_residual_mm: float = float("nan")

    def sensor_to_base_matrix(self) -> np.ndarray:
        """[scale * R, s; 0 0 0 1]."""
        matrix = self.sensor_to_base.as_matrix()
        matrix[:3, :3] *= self.scale
        return matrix

    def solved(self) -> bool:
        return self.status in (RegistrationStatus.SUCCESS, RegistrationStatus.RESIDUALS_EXCEED_THRESHOLD)


# ---------------------------------------------------------------------------
# Building blocks
# ---------------------------------------------------------------------------
def spread(matrix: np.ndarray) -> float:
    """Smallest singular value of an N-row matrix divided by sqrt(N): independent of N for rows of
    unit scale; zero when the columns are linearly dependent (see RegistrationParameters)."""
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape[0] == 0:
        return 0.0
    return float(np.linalg.svd(matrix, compute_uv=False)[-1] / np.sqrt(matrix.shape[0]))


def solve_rotation_between_directions(from_directions, to_directions, weights=None) -> np.ndarray:
    """Rotation R minimizing sum_k w_k |R from_k - to_k|^2 over proper rotations: orthogonal
    Procrustes on direction vectors, H = sum w_k from_k to_k^t, H = U S V^t, R = V D U^t with
    D = diag(1, 1, det(V U^t)). No centroid subtraction: the inputs are directions, not points
    (REVIEW.md Section 4.5). Needs at least two non-parallel directions."""
    source = np.asarray(from_directions, dtype=np.float64).reshape(-1, 3)
    target = np.asarray(to_directions, dtype=np.float64).reshape(-1, 3)
    if weights is None:
        weights = np.ones(source.shape[0])
    weights = np.asarray(weights, dtype=np.float64).reshape(-1)
    cross_covariance = (source * weights[:, None]).T @ target
    u, _, vt = np.linalg.svd(cross_covariance)
    correction = np.diag([1.0, 1.0, np.sign(np.linalg.det(vt.T @ u.T)) or 1.0])
    return vt.T @ correction @ u.T


def _solve_translation(base_normals, base_offsets, sensor_offsets) -> np.ndarray:
    """Rigid: least squares M s = d_k - e_k (REVIEW.md Section 6 step 5)."""
    solution, *_ = np.linalg.lstsq(base_normals, sensor_offsets - base_offsets, rcond=None)
    return solution


def _solve_translation_and_scale(base_normals, base_offsets, sensor_offsets) -> tuple[np.ndarray, float]:
    """Similarity: least squares [m_k, -d_k] (s, c) = -e_k (REVIEW.md Section 4.7)."""
    system = np.column_stack([base_normals, -sensor_offsets])
    solution, *_ = np.linalg.lstsq(system, -base_offsets, rcond=None)
    return solution[:3], float(solution[3])


def _residuals(rotation, translation, scale, sensor_planes, base_planes) -> list[tuple[float, float]]:
    """(normal angle in degrees, offset residual) of every pose for the given solution."""
    out = []
    for sensor, base in zip(sensor_planes, base_planes):
        rotated = rotation @ sensor.normal
        sine = float(np.linalg.norm(np.cross(rotated, base.normal)))
        cosine = float(rotated @ base.normal)
        angle = float(np.degrees(np.arctan2(sine, cosine)))
        offset = float(translation @ base.normal) + base.offset - scale * sensor.offset
        out.append((angle, offset))
    return out


def _solve_once(sensor_planes, base_planes, params: RegistrationParameters):
    """One solve over the given (already normalized) planes. Returns a partially filled
    RegistrationResult with status SUCCESS when a transform was found."""
    base_normals = np.array([p.normal for p in base_planes])
    base_offsets = np.array([p.offset for p in base_planes])
    sensor_normals = np.array([p.normal for p in sensor_planes])
    sensor_offsets = np.array([p.offset for p in sensor_planes])

    result = RegistrationResult(RegistrationStatus.SUCCESS, "")
    result.normal_spread = spread(base_normals)
    if result.normal_spread < params.minimum_normal_spread:
        result.status = RegistrationStatus.DEGENERATE_POSES
        result.message = (f"normal spread {result.normal_spread:.4f} is below the minimum "
                          f"{params.minimum_normal_spread:.4f}: tilt the plate more between poses")
        return result

    rotation = solve_rotation_between_directions(sensor_normals, base_normals)
    if params.transform_model == TransformModel.RIGID:
        translation, scale = _solve_translation(base_normals, base_offsets, sensor_offsets), 1.0
    else:
        largest_offset = float(np.max(np.abs(sensor_offsets)))
        if largest_offset <= 0.0:
            result.status = RegistrationStatus.DEGENERATE_POSES
            result.message = "all sensor offsets are zero, so the scale is unobservable"
            return result
        result.similarity_spread = spread(np.column_stack([base_normals, -sensor_offsets / largest_offset]))
        if result.similarity_spread < params.minimum_similarity_spread:
            result.status = RegistrationStatus.DEGENERATE_POSES
            result.message = (f"similarity spread {result.similarity_spread:.4f} is below the minimum "
                              f"{params.minimum_similarity_spread:.4f}: vary the standoff between poses")
            return result
        translation, scale = _solve_translation_and_scale(base_normals, base_offsets, sensor_offsets)
        if not scale > 0.0:
            result.status = RegistrationStatus.DEGENERATE_POSES
            result.message = f"the recovered scale {scale:.6f} is not positive"
            return result
    result.sensor_to_base = RigidTransform(rotation, translation)
    result.scale = scale
    return result


# ---------------------------------------------------------------------------
# The registration
# ---------------------------------------------------------------------------
def register_planes(sensor_planes: list[Plane], base_planes: list[Plane],
                    params: RegistrationParameters = RegistrationParameters()) -> RegistrationResult:
    """Solve the sensor-to-base transform from paired planes (REVIEW.md Section 6).

    ``sensor_planes[k]`` is the plane measured by the sensor at pose k (any scale or sign; it is
    normalized here to a unit normal with d > 0), ``base_planes[k]`` the plate plane in the base
    frame from the reported pose. Returns a RegistrationResult whose status says whether the
    transform is valid (SUCCESS or RESIDUALS_EXCEED_THRESHOLD) and whose residuals list has one
    entry per input pose, in order, used or not.
    """
    if len(sensor_planes) != len(base_planes):
        return RegistrationResult(RegistrationStatus.INVALID_INPUT,
                                  f"{len(sensor_planes)} sensor planes but {len(base_planes)} base planes")
    try:
        sensor = [p.normalized(origin_on_positive_side=True) for p in sensor_planes]
        base = [p.unit() for p in base_planes]   # orientation already meaningful: flange +z
    except ValueError as error:
        return RegistrationResult(RegistrationStatus.INVALID_INPUT, str(error))

    model_floor = (RIGID_MINIMUM_POSE_COUNT if params.transform_model == TransformModel.RIGID
                   else SIMILARITY_MINIMUM_POSE_COUNT)
    required = max(model_floor, params.minimum_pose_count)
    used = [True] * len(sensor)
    final: RegistrationResult | None = None
    for _round in range(max(params.outlier_rejection_rounds, 0) + 1):
        kept = [k for k, flag in enumerate(used) if flag]
        if len(kept) < required:
            return RegistrationResult(RegistrationStatus.TOO_FEW_POSES,
                                      f"{len(kept)} usable poses, but {required} are required")
        result = _solve_once([sensor[k] for k in kept], [base[k] for k in kept], params)
        if result.status != RegistrationStatus.SUCCESS:
            return result
        pairs = _residuals(result.sensor_to_base.rotation, result.sensor_to_base.translation, result.scale,
                           sensor, base)
        result.residuals = [PoseResidual(angle, offset, used[k]) for k, (angle, offset) in enumerate(pairs)]
        final = result
        # Drop the single worst pose relative to its thresholds, if any pose exceeds them and the
        # floor allows it (one per round: leverage, see RegistrationParameters).
        worst_index, worst_ratio = None, OUTLIER_RATIO_LIMIT
        for k in kept:
            angle, offset = pairs[k]
            ratio = max(angle / params.outlier_normal_residual_degrees,
                        abs(offset) / params.outlier_offset_residual_mm)
            if ratio > worst_ratio:
                worst_index, worst_ratio = k, ratio
        if worst_index is None or len(kept) - 1 < required:
            break
        used[worst_index] = False

    assert final is not None
    angles = np.array([r.normal_angle_degrees for r in final.residuals if r.used])
    offsets = np.array([abs(r.offset_residual_mm) for r in final.residuals if r.used])
    final.poses_used = int(angles.size)
    final.rms_normal_residual_degrees = float(np.sqrt(np.mean(angles ** 2)))
    final.max_normal_residual_degrees = float(angles.max())
    final.rms_offset_residual_mm = float(np.sqrt(np.mean(offsets ** 2)))
    final.max_offset_residual_mm = float(offsets.max())
    accepted = (final.rms_normal_residual_degrees <= params.maximum_rms_normal_residual_degrees
                and final.rms_offset_residual_mm <= params.maximum_rms_offset_residual_mm
                and final.max_normal_residual_degrees <= params.maximum_normal_residual_degrees
                and final.max_offset_residual_mm <= params.maximum_offset_residual_mm)
    final.status = RegistrationStatus.SUCCESS if accepted else RegistrationStatus.RESIDUALS_EXCEED_THRESHOLD
    final.message = ("registration succeeded" if accepted
                     else "registration solved, but a residual exceeds its acceptance threshold")
    return final


def expected_registration_error(normal_spread_value: float, pose_count: int,
                                normal_noise_degrees: float, offset_noise_mm: float) -> tuple[float, float]:
    """Rough prediction of the registration error (rotation RMS in degrees, translation RMS in mm)
    for a planned pose set, from the review's synthetic model (REVIEW.md Section 4.6, check 9):
    each component of the solution is a least-squares average whose standard error scales as
    noise / (spread * sqrt(N)). The constant is 1 (no fudge); treat the result as an order of
    magnitude, good to a factor of about two."""
    if normal_spread_value <= 0.0 or pose_count <= 0:
        return float("inf"), float("inf")
    denominator = normal_spread_value * np.sqrt(pose_count)
    return normal_noise_degrees / denominator, offset_noise_mm / denominator
