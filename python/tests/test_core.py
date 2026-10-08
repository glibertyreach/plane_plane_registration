"""
Tests of planereg.core: the registration solver, plane algebra and the target-plane
segmentation, on synthetic boards rendered with sphcal's renderer (design document, Section 9).
"""
from __future__ import annotations

import numpy as np
import pytest
from scipy import ndimage
from scipy.spatial.transform import Rotation

from planereg.core.planes import Plane, board_plane_in_base, predicted_sensor_plane, transform_plane
from planereg.core.registration import (RegistrationParameters, RegistrationStatus, TransformModel,
                                        register_planes, residual_statistics, residuals_of_planes, spread,
                                        within_acceptance_limits)
from planereg.core.segmentation import SegmentationParameters, board_prediction, segment_target_plane
from sphcal.geometry.camera import PinholeCamera
from sphcal.geometry.transforms import RigidTransform
from sphcal.simulate.synthetic import SyntheticSensorParameters, render_board_frame

TEST_CAMERA = PinholeCamera(320, 240, 344.08, 344.04, 146.16, 128.38)
"""A quarter-size version of the indicative sensor, so each test runs in well under a second."""
TEST_BLOCK_PX = 2
NO_NOREAD_ONSET_DEG = 90.0
NOISELESS_SENSOR = SyntheticSensorParameters(TEST_CAMERA, TEST_BLOCK_PX, 0.0, 1.3, NO_NOREAD_ONSET_DEG, 5.0, 0.0, 0)
"""Renderer with no noise and (in practice) no no-reads: exact geometry."""
NOISE_COEFFICIENT_PER_MM = 1.79e-7
NOISE_INCIDENCE_EXPONENT = 1.3
NOREAD_ONSET_DEG = 55.0
NOREAD_WIDTH_DEG = 5.0
NOISY_SENSOR = SyntheticSensorParameters(TEST_CAMERA, TEST_BLOCK_PX, NOISE_COEFFICIENT_PER_MM,
                                         NOISE_INCIDENCE_EXPONENT, NOREAD_ONSET_DEG, NOREAD_WIDTH_DEG, 0.0, 0)
"""Renderer with the indicative noise model of the real sensor (sphcal's values)."""
BOARD_HALF_SIZE_MM = (100.0, 75.0)
SENSOR_TO_BASE = RigidTransform.from_rotation_vector_degrees([170.0, 5.0, -3.0], [900.0, -50.0, 1200.0])
"""A sensor looking roughly down on the workspace from 1.2 m (base z up)."""
SEED = 20250616
POSE_COUNT = 16
LATERAL_HALF_RANGE_MM = (90.0, 60.0)
STANDOFF_RANGE_MM = (500.0, 850.0)
TILT_HALF_RANGE_DEG = 30.0
EXACT_ROTATION_TOL_DEG = 1e-7
EXACT_TRANSLATION_TOL_MM = 1e-6
HELD_OUT_OFFSET_ERROR_MM = 8.0
"""Offset error given to the plane that is evaluated without being fitted."""
NOISY_ROTATION_TOL_DEG = 0.05
NOISY_TRANSLATION_TOL_MM = 0.5
INJECTED_SCALE = 1.02
SCALE_TOL = 1e-3
WALL_BEHIND_MM = 300.0
PATCH_HALF_SIZE_MM = (30.0, 30.0)
PATCH_IN_FRONT_MM = 150.0
FLYAWAY_FRACTION = 0.3
FLYAWAY_BAND_PX = 2
FLYAWAY_FACTOR_RANGE = (0.9, 1.1)
FLYAWAY_ANGLE_TOL_DEG = 0.02
FLYAWAY_OFFSET_TOL_MM = 0.05
ROUGH_ROTATION_ERROR_DEG = 2.0
ROUGH_TRANSLATION_ERROR_MM = 20.0
ORTHOGONAL_SPREAD = 1.0 / np.sqrt(3.0)


def zero_error_field(u, v, rho, s_u, s_v, curvature):
    return np.zeros_like(u)


def random_board_poses(rng: np.random.Generator, count: int) -> list[RigidTransform]:
    """Board tool frames in the SENSOR frame: centers spread laterally and in standoff, facing
    the sensor with random tilts about both in-plane axes."""
    poses = []
    for _ in range(count):
        center = np.array([rng.uniform(-LATERAL_HALF_RANGE_MM[0], LATERAL_HALF_RANGE_MM[0]),
                           rng.uniform(-LATERAL_HALF_RANGE_MM[1], LATERAL_HALF_RANGE_MM[1]),
                           rng.uniform(*STANDOFF_RANGE_MM)])
        toward_sensor = -center / np.linalg.norm(center)
        tilt = (Rotation.from_rotvec(np.radians(rng.uniform(-TILT_HALF_RANGE_DEG, TILT_HALF_RANGE_DEG)) * np.array([1.0, 0.0, 0.0]))
                * Rotation.from_rotvec(np.radians(rng.uniform(-TILT_HALF_RANGE_DEG, TILT_HALF_RANGE_DEG)) * np.array([0.0, 1.0, 0.0])))
        z_axis = tilt.apply(toward_sensor)
        x_axis = np.cross([0.0, 1.0, 0.0], z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        poses.append(RigidTransform(np.column_stack([x_axis, y_axis, z_axis]), center))
    return poses


def render_points(sensor: SyntheticSensorParameters, board_to_sensor: RigidTransform, rng) -> np.ndarray:
    frame = render_board_frame(sensor, board_to_sensor, BOARD_HALF_SIZE_MM, zero_error_field, rng)
    return np.where(frame.read_mask[..., None], frame.xyz.astype(np.float64), np.nan)


def rotation_error_degrees(a: np.ndarray, b: np.ndarray) -> float:
    difference = a.T @ b - np.eye(3)
    return float(np.degrees(2.0 * np.arcsin(min(1.0, np.linalg.norm(difference) / (2.0 * np.sqrt(2.0))))))


# ---------------------------------------------------------------------------
# Plane algebra
# ---------------------------------------------------------------------------
def test_transform_plane_keeps_points_on_the_plane():
    rng = np.random.default_rng(SEED)
    transform = RigidTransform.from_rotation_vector_degrees(rng.uniform(-90, 90, 3), rng.uniform(-500, 500, 3))
    plane = Plane(rng.normal(size=3), rng.uniform(-100, 100)).normalized()
    basis = np.linalg.svd(plane.normal[None, :])[2][1:]
    points_on_plane = -plane.offset * plane.normal + rng.normal(size=(5, 1)) * basis[0] + rng.normal(size=(5, 1)) * basis[1]
    moved = transform_plane(transform, plane)
    assert np.abs(moved.signed_distance(transform.apply_points(points_on_plane))).max() < 1e-9
    assert abs(np.linalg.norm(moved.normal) - 1.0) < 1e-12


def test_board_plane_and_prediction_are_consistent():
    rng = np.random.default_rng(SEED)
    board_to_sensor = random_board_poses(rng, 1)[0]
    board_to_base = SENSOR_TO_BASE.compose(board_to_sensor)
    base_plane = board_plane_in_base(board_to_base)
    predicted = predicted_sensor_plane(SENSOR_TO_BASE, base_plane).normalized()
    # The board center, in the sensor frame, lies on the predicted sensor-frame plane, and the
    # predicted normal is the board's +z axis.
    assert abs(predicted.signed_distance(board_to_sensor.translation)) < 1e-9
    assert np.allclose(predicted.normal, board_to_sensor.rotation[:, 2])


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("model", [TransformModel.RIGID, TransformModel.SIMILARITY])
def test_registration_recovers_exact_transform(model):
    rng = np.random.default_rng(SEED)
    scale = INJECTED_SCALE if model == TransformModel.SIMILARITY else 1.0
    sensor_planes, base_planes = [], []
    for board_to_sensor in random_board_poses(rng, POSE_COUNT):
        base_plane = board_plane_in_base(SENSOR_TO_BASE.compose(board_to_sensor))
        base_planes.append(base_plane)
        sensor_planes.append(predicted_sensor_plane(SENSOR_TO_BASE, base_plane, scale))
    result = register_planes(sensor_planes, base_planes, RegistrationParameters(transform_model=model))
    assert result.status == RegistrationStatus.SUCCESS, result.message
    assert rotation_error_degrees(result.sensor_to_base.rotation, SENSOR_TO_BASE.rotation) < EXACT_ROTATION_TOL_DEG
    assert np.linalg.norm(result.sensor_to_base.translation - SENSOR_TO_BASE.translation) < EXACT_TRANSLATION_TOL_MM
    assert abs(result.scale - scale) < SCALE_TOL


def test_registration_rejects_a_single_outlier_pose():
    rng = np.random.default_rng(SEED)
    sensor_planes, base_planes = [], []
    for board_to_sensor in random_board_poses(rng, POSE_COUNT):
        base_plane = board_plane_in_base(SENSOR_TO_BASE.compose(board_to_sensor))
        base_planes.append(base_plane)
        sensor_planes.append(predicted_sensor_plane(SENSOR_TO_BASE, base_plane))
    bad = sensor_planes[3]
    sensor_planes[3] = Plane(bad.normal, bad.offset + 8.0)
    result = register_planes(sensor_planes, base_planes, RegistrationParameters(outlier_rejection_rounds=3))
    assert result.status == RegistrationStatus.SUCCESS, result.message
    assert [k for k, r in enumerate(result.residuals) if not r.used] == [3]
    assert np.linalg.norm(result.sensor_to_base.translation - SENSOR_TO_BASE.translation) < 1e-5


def test_residuals_of_unfitted_planes_use_the_solvers_own_residuals():
    """Planes evaluated against a solved transform get the residuals the solver gives them as fitted poses, and a
    plane with an offset error shows that error; the statistics and the verdict use the acceptance limits."""
    rng = np.random.default_rng(SEED)
    sensor_planes, base_planes = [], []
    for board_to_sensor in random_board_poses(rng, POSE_COUNT):
        base_plane = board_plane_in_base(SENSOR_TO_BASE.compose(board_to_sensor))
        base_planes.append(base_plane)
        sensor_planes.append(predicted_sensor_plane(SENSOR_TO_BASE, base_plane))
    sensor_planes[2] = Plane(sensor_planes[2].normal, sensor_planes[2].offset + HELD_OUT_OFFSET_ERROR_MM)
    result = register_planes(sensor_planes, base_planes)
    evaluated = residuals_of_planes(result, sensor_planes, base_planes)
    assert [r.used for r in evaluated] == [False] * POSE_COUNT
    for fitted, again in zip(result.residuals, evaluated):
        assert again.normal_angle_degrees == pytest.approx(fitted.normal_angle_degrees, abs=1e-12)
        assert again.offset_residual_mm == pytest.approx(fitted.offset_residual_mm, abs=1e-12)
    # The same planes against the transform of a fit that never saw plane 2 show its error in full.
    clean = [k for k in range(POSE_COUNT) if k != 2]
    clean_result = register_planes([sensor_planes[k] for k in clean], [base_planes[k] for k in clean])
    held_out = residuals_of_planes(clean_result, [sensor_planes[2]], [base_planes[2]])[0]
    assert abs(abs(held_out.offset_residual_mm) - HELD_OUT_OFFSET_ERROR_MM) < EXACT_TRANSLATION_TOL_MM
    statistics = residual_statistics([held_out])
    limits = RegistrationParameters()
    assert statistics.rms_offset_mm == pytest.approx(abs(held_out.offset_residual_mm))
    assert not within_acceptance_limits(statistics, limits)
    assert within_acceptance_limits(residual_statistics(
        residuals_of_planes(clean_result, [sensor_planes[k] for k in clean], [base_planes[k] for k in clean])), limits)
    assert np.isnan(residual_statistics([]).rms_normal_degrees)
    with pytest.raises(ValueError):
        residuals_of_planes(register_planes(sensor_planes[:1], base_planes[:1]), sensor_planes, base_planes)


def test_parallel_normals_are_degenerate_and_orthogonal_spread_is_known():
    assert abs(spread(np.eye(3)) - ORTHOGONAL_SPREAD) < 1e-12
    base_planes = [Plane([0.0, 0.0, 1.0], -k * 10.0) for k in range(5)]
    sensor_planes = [Plane([0.0, 0.0, 1.0], 100.0 + k * 10.0) for k in range(5)]
    result = register_planes(sensor_planes, base_planes)
    assert result.status == RegistrationStatus.DEGENERATE_POSES


# ---------------------------------------------------------------------------
# Segmentation
# ---------------------------------------------------------------------------
def test_segmentation_prefers_the_board_over_wall_and_small_patch():
    rng = np.random.default_rng(SEED)
    board_to_sensor = random_board_poses(rng, 1)[0]
    points = render_points(NOISELESS_SENSOR, board_to_sensor, rng)
    # A wall 300 mm behind the board center, filling the field, and a small patch in front.
    rays = TEST_CAMERA.ray_directions()
    wall_z = board_to_sensor.translation[2] + WALL_BEHIND_MM
    wall = rays * (wall_z / rays[..., 2])[..., None]
    points = np.where(np.isnan(points), wall, points)
    patch_pose = RigidTransform(np.eye(3), board_to_sensor.translation - np.array([0.0, 0.0, PATCH_IN_FRONT_MM]))
    patch = render_board_frame(NOISELESS_SENSOR, patch_pose, PATCH_HALF_SIZE_MM, zero_error_field, rng)
    points = np.where(patch.read_mask[..., None], patch.xyz.astype(np.float64), points)

    result = segment_target_plane(points, TEST_CAMERA, SegmentationParameters())
    assert result.ok, result.message
    truth = Plane(board_to_sensor.rotation[:, 2], -float(board_to_sensor.rotation[:, 2] @ board_to_sensor.translation)).normalized()
    assert result.plane.angle_to_degrees(truth) < 0.01
    assert abs(result.plane.offset - truth.offset) < 0.02
    assert result.candidate_count >= 2   # the wall was found too, and rejected as farther


def test_flyaways_on_the_silhouette_do_not_move_the_plane():
    rng = np.random.default_rng(SEED)
    board_to_sensor = random_board_poses(rng, 1)[0]
    clean = render_points(NOISY_SENSOR, board_to_sensor, rng)
    reference = segment_target_plane(clean, TEST_CAMERA, SegmentationParameters())
    assert reference.ok
    valid = np.isfinite(clean[..., 2])
    band = valid & ~ndimage.binary_erosion(valid, iterations=FLYAWAY_BAND_PX)
    flyaway = band & (rng.uniform(size=band.shape) < FLYAWAY_FRACTION)
    dirty = clean.copy()
    dirty[flyaway] *= rng.uniform(*FLYAWAY_FACTOR_RANGE, size=(int(flyaway.sum()), 1))
    result = segment_target_plane(dirty, TEST_CAMERA, SegmentationParameters())
    assert result.ok, result.message
    assert result.plane.angle_to_degrees(reference.plane) < FLYAWAY_ANGLE_TOL_DEG
    assert abs(result.plane.offset - reference.plane.offset) < FLYAWAY_OFFSET_TOL_MM


def test_predicted_region_tolerates_a_rough_transform():
    rng = np.random.default_rng(SEED)
    board_to_sensor = random_board_poses(rng, 1)[0]
    board_to_base = SENSOR_TO_BASE.compose(board_to_sensor)
    points = render_points(NOISELESS_SENSOR, board_to_sensor, rng)
    error = RigidTransform.from_rotation_vector_degrees([ROUGH_ROTATION_ERROR_DEG, 0.0, 0.0],
                                                        [ROUGH_TRANSLATION_ERROR_MM, 0.0, 0.0])
    rough = error.compose(SENSOR_TO_BASE)
    prediction = board_prediction(rough, board_to_base, BOARD_HALF_SIZE_MM)
    result = segment_target_plane(points, TEST_CAMERA, SegmentationParameters(), prediction)
    assert result.ok, result.message
    assert result.method == "predicted_region"
    truth = Plane(board_to_sensor.rotation[:, 2], -float(board_to_sensor.rotation[:, 2] @ board_to_sensor.translation)).normalized()
    assert result.plane.angle_to_degrees(truth) < 0.01


def test_end_to_end_registration_from_rendered_boards():
    rng = np.random.default_rng(SEED)
    sensor_planes, base_planes = [], []
    for board_to_sensor in random_board_poses(rng, POSE_COUNT):
        points = render_points(NOISY_SENSOR, board_to_sensor, rng)
        segmentation = segment_target_plane(points, TEST_CAMERA, SegmentationParameters())
        assert segmentation.ok, segmentation.message
        sensor_planes.append(segmentation.plane)
        base_planes.append(board_plane_in_base(SENSOR_TO_BASE.compose(board_to_sensor)))
    result = register_planes(sensor_planes, base_planes)
    assert result.solved(), result.message
    assert rotation_error_degrees(result.sensor_to_base.rotation, SENSOR_TO_BASE.rotation) < NOISY_ROTATION_TOL_DEG
    assert np.linalg.norm(result.sensor_to_base.translation - SENSOR_TO_BASE.translation) < NOISY_TRANSLATION_TOL_MM
