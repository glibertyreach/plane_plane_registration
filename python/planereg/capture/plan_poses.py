"""
Command line: plan the robot poses of a registration session (code design, plan tool).

    python3 -m planereg.capture.plan_poses --sensor-in-base sensor_in_base.json \
        --camera capture.mc --out plan/

What it computes
----------------
Boards at several standoffs, lateral positions, tilts and tilt azimuths, laid out in the SENSOR
frame (so that every board is inside the image whatever the robot does) and carried into the robot
BASE frame with the rough sensor-to-base transform (the bootstrap tool's JSON, or any file in
sphcal's ``--sensor-in-base`` format).

Frames and conventions (millimeters, degrees at the interface)
    sensor frame S   z along the optical axis, x right, y down.
    base frame B     the robot base frame; every commanded pose is in it.
    board frame T    origin at the center of the board's front face, x along the long edge, y along
                     the short edge, z out of the face toward the sensor (the robot's tool frame
                     when the board tool frame is active). The target pose of a plan row is T -> B.
    standoff         the sensor-frame z (depth) of the board center.
    lateral position the board center is at x = f_x * lateral_fill * half_x(z) and
                     y = f_y * lateral_fill * half_y(z), where half_x(z) and half_y(z) are the
                     half extents of the field of view at depth z and f_x, f_y run over an evenly
                     spaced set of ``lateral_positions`` values in [-1, +1] (a grid of
                     nx x ny positions; with an odd count one position sits at 0).
    orientation      z points from the board center toward the sensor origin (the board "faces"
                     the sensor), x is the sensor x axis made perpendicular to z; the board is then
                     tilted by the tilt angle about the in-plane axis at the azimuth (azimuth 0 is
                     the board x axis, 90 the board y axis; seen in the sensor's picture, azimuth
                     0 brings the top edge toward the sensor, 90 the right edge, 180 the bottom
                     edge, 270 the left edge). A tilt of zero is planned once per lateral
                     position, with azimuth 0.
    drop rule        a pose whose four corners do not all project at least ``edge_margin_px`` inside
                     the image is dropped and counted.
    order            standoff, then lateral position (serpentine over the grid so that consecutive
                     poses are neighbors), then tilt and azimuth.

The two sub-procedures and the approach pose (design Section 6)
    Every row carries, besides the target pose T, an approach pose A, also T-frame -> B:

        A = T o D,    D(x) = Rx(approach_rotation_deg) x + (0, 0, -approach_retreat_mm)

    that is, with R_T and t_T the rotation and position of the target pose,

        R_A = R_T Rx(approach_rotation_deg)          (rotation about the board's own x axis)
        t_A = t_T + R_T (0, 0, -approach_retreat_mm) (retreat along the board's own -z, away from the sensor)

    so that A^-1 T = D^-1 is the same displacement in the tool frame for every pose. Sub-procedure A
    (ignoring backlash) goes straight to T; sub-procedure B goes to A by a joint move and then onto T
    by a linear move. Both run the same poses.csv.

Outputs in --out
    poses.csv         sphcal's POSE_CSV_COLUMNS (so ``sphcal.cli.make_manifest`` accepts it as a pose
                      log unchanged) followed by standoff_mm, tilt_deg, azimuth_deg,
                      approach_x_mm, approach_y_mm, approach_z_mm, approach_r00..approach_r22
                      (the approach pose in B, row-major rotation), approach_quat_w..approach_quat_z
                      (the same rotation as a unit quaternion with w >= 0) and
                      approach_rotvec_x_deg..approach_rotvec_z_deg (the same rotation as a rotation
                      vector in degrees). The quaternion and the rotation vector are computed in the
                      same way as the target pose's quat_* and rotvec_* columns, so that a controller
                      that takes either form can be fed the approach pose too.
    plan_summary.txt  counts per standoff and tilt, dropped poses, normal spread and similarity spread
                      of the planned set, predicted registration error, capture counts.
    plan.png          side and front views of the board centers and outlines in the field of view.

Exit code: 0 when the plan was written, 2 when an input must be fixed (the message says what).
"""
from __future__ import annotations

import argparse
import csv
import itertools
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from planereg.capture.common import EXIT_INPUT_ERROR, EXIT_OK, load_rough_transform, print_error, print_warning
from planereg.core.planes import board_plane_in_base
from planereg.core.registration import RegistrationParameters, expected_registration_error, spread
from sphcal.cli.plan_poses import (BOARD_CORNER_SIGNS, MAX_BOARD_TILT_DEG, POSE_CSV_COLUMNS, PlanInputError,
                                   PlannedPose, board_corners_in_view, camera_from_arguments, frame_from_z_axis,
                                   half_field_at_depth_mm, pose_row)
from sphcal.geometry.camera import PinholeCamera
from sphcal.geometry.transforms import RigidTransform
from sphcal.io.poses import TARGET_KIND_BOARD

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
PLAN_CSV_NAME = "poses.csv"
PLAN_SUMMARY_NAME = "plan_summary.txt"
PLAN_FIGURE_NAME = "plan.png"
"""File names written into the output directory."""

PLAN_EXTRA_COLUMNS = (
    ("standoff_mm", "tilt_deg", "azimuth_deg", "approach_x_mm", "approach_y_mm", "approach_z_mm")
    + tuple(f"approach_r{row}{col}" for row in range(3) for col in range(3))
    + ("approach_quat_w", "approach_quat_x", "approach_quat_y", "approach_quat_z",
       "approach_rotvec_x_deg", "approach_rotvec_y_deg", "approach_rotvec_z_deg"))
"""Columns of poses.csv after sphcal's POSE_CSV_COLUMNS: the plan coordinates of the pose and the
approach pose in the base frame (position, the rotation matrix row-major, the unit quaternion
w, x, y, z with w >= 0, and the rotation vector in degrees). The target pose has the same three
rotation forms in sphcal's columns."""

BOARD_X_AXIS = np.eye(3)[0]
"""The board's x axis (long edge) in the board frame; the approach rotation is about it."""
BOARD_Z_AXIS = np.eye(3)[2]
"""The board's z axis (face normal, toward the sensor) in the board frame; the approach retreat is
along its negative."""

UNTILTED_AZIMUTH_DEG = 0.0
"""Azimuth written for a pose of tilt zero (the azimuth has no meaning there; the pose is planned once)."""
LATERAL_FRACTION_RANGE = (-1.0, 1.0)
"""Lowest and highest lateral fraction; the fractions of a grid axis are evenly spaced between them."""
BOARD_ID_FORMAT = "b_z{standoff:04.0f}_t{tilt:02.0f}_a{azimuth:03.0f}_x{ix}y{iy}"
"""Pose id: standoff, tilt, azimuth, and the lateral grid indices (column ix, row iy). Pose ids that
end in digits need a frame suffix in the capture file name, e.g. ``<pose_id>_Index00.mc``."""

MAX_LATERAL_GRID_SIZE = 20
"""Largest number of lateral positions per axis that --target-pose-count will consider."""
MAX_GRID_ASPECT_DIFFERENCE = 1
"""--target-pose-count considers grids whose column and row counts differ by at most this."""
POSE_COUNT_WARN = 1000
"""Design Section 6: a plan above this many poses is made only if the first run's residual analysis
calls for it, so the plan tool warns."""
MIN_FRAMES_PER_POSE = 1
SUB_PROCEDURE_COUNT = 2
"""Sub-procedures A and B each capture the whole plan."""

PLOT_DPI = 130
"""Resolution of plan.png."""
PLOT_FIGURE_SIZE_IN = (12.0, 6.0)
"""Figure size of plan.png in inches (width, height)."""
PLOT_STANDOFF_COLORS = ("#0072B2", "#D55E00", "#009E73", "#CC79A7", "#56B4E9", "#E69F00", "#F0E442")
"""One hue per standoff (Okabe-Ito, colorblind safe); cycled when there are more standoffs."""
PLOT_FRUSTUM_COLOR = "#444444"
"""Color of the field-of-view lines."""
PLOT_HOLDOUT_EDGE_COLOR = "#000000"
"""Ring color of the center marker of a held-out pose."""
PLOT_OUTLINE_LINE_WIDTH = 1.2
PLOT_FRUSTUM_LINE_WIDTH = 1.0
PLOT_HOLDOUT_LINE_WIDTH = 1.6
PLOT_MARKER_SIZE = 14.0
"""Line widths (points) and scatter marker area (points squared) of the figure."""
PLOT_HOLDOUT_OUTLINE_STYLE = "--"
PLOT_KEPT_OUTLINE_STYLE = "-"
"""Held-out boards are outlined dashed, the others solid."""
PLOT_GRID_ALPHA = 0.3
"""Transparency of the figure's grid lines."""
PLOT_OUTLINE_ALPHA = 0.75
"""Transparency of the board outlines, so that overlapping boards of the front view stay readable."""


# ---------------------------------------------------------------------------
# Parameters and records
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class PlanParameters:
    """Every number of the plan; the command-line defaults come from here. The defaults give about
    240 poses with a 640 x 480 indicative camera (design Section 6, the pose budget: a first run of a
    few hundred poses per sub-procedure)."""

    board_half_size_mm: tuple[float, float] = (100.0, 75.0)
    """Board half width (along board x) and half height (along board y)."""
    standoffs_mm: tuple[float, ...] = (550.0, 750.0, 950.0)
    """Sensor-z depths of the board centers. At 450 mm the 200 x 150 mm board already fills so much
    of the 640 x 480 sensor's field that most tilted poses would leave the image."""
    tilts_deg: tuple[float, ...] = (0.0, 15.0, 30.0)
    """Board tilts away from facing the sensor; tilt zero is planned once per lateral position."""
    azimuths_deg: tuple[float, ...] = (0.0, 90.0, 180.0, 270.0)
    """Azimuths of the tilt axis in the board plane (0 = board x axis, 90 = board y axis)."""
    lateral_fill: float = 0.5
    """Fraction of the half field covered by board centers at each standoff."""
    lateral_positions: tuple[int, int] = (3, 3)
    """Board center positions across the field at each standoff: columns (x) and rows (y)."""
    edge_margin_px: float = 10.0
    """Board corners must project at least this far inside the image."""
    approach_retreat_mm: float = 40.0
    """Retreat of the approach pose along the board's own -z."""
    approach_rotation_deg: float = 3.0
    """Rotation of the approach pose about the board's own x axis."""
    holdout_fraction: float = 0.2
    """Fraction of the poses tagged as held out (informational: the analysis may leave them out of the fit)."""
    seed: int = 0
    """Seed of the random held-out subset."""
    target_pose_count: int | None = None
    """When given, the lateral grid is chosen so that the plan has about this many poses."""
    noise_normal_deg: float = 0.2
    """Assumed noise of a measured plane normal, for the error prediction."""
    noise_offset_mm: float = 0.2
    """Assumed noise of a measured plane offset, for the error prediction."""
    frames_per_pose: int = 5
    """Frames captured at every pose (procedure document); only used for the capture counts."""
    bootstrap_residual_warn_mm: float = 5.0
    """Warn when a sensor/base point pair of a sphcal-style bootstrap file disagrees by more than this."""


@dataclass
class SensorPose:
    """One planned board pose, in the sensor frame."""

    pose_id: str
    standoff_mm: float
    tilt_deg: float
    azimuth_deg: float
    center_sensor: np.ndarray       # (3,) board center, mm
    board_to_sensor: np.ndarray     # (3, 3) rotation T -> S
    holdout: bool = False


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------
def validate_parameters(p: PlanParameters) -> None:
    """Raise PlanInputError, saying what to change, for inconsistent parameters."""
    checks = [
        (len(p.board_half_size_mm) == 2 and all(h > 0 for h in p.board_half_size_mm),
         "--board-half-size-mm needs two positive numbers"),
        (len(p.standoffs_mm) > 0 and all(s > 0 for s in p.standoffs_mm), "--standoffs-mm must be positive numbers"),
        (len(p.tilts_deg) > 0 and all(0 <= t < MAX_BOARD_TILT_DEG for t in p.tilts_deg),
         f"--tilts-deg must be in [0, {MAX_BOARD_TILT_DEG:g})"),
        (len(p.azimuths_deg) > 0, "give at least one azimuth in --azimuths-deg"),
        (0 <= p.lateral_fill <= 1, "--lateral-fill must be in [0, 1]"),
        (all(count >= 1 for count in p.lateral_positions), "--lateral-positions needs two counts of at least 1"),
        (p.edge_margin_px >= 0, "--edge-margin-px must not be negative"),
        (p.approach_retreat_mm >= 0, "--approach-retreat-mm must not be negative"),
        (0 <= p.holdout_fraction <= 1, "--holdout-fraction must be in [0, 1]"),
        (p.target_pose_count is None or p.target_pose_count >= 1, "--target-pose-count must be at least 1"),
        (p.noise_normal_deg >= 0 and p.noise_offset_mm >= 0, "--noise-normal-deg and --noise-offset-mm must not be negative"),
        (p.frames_per_pose >= MIN_FRAMES_PER_POSE, f"--frames-per-pose must be at least {MIN_FRAMES_PER_POSE}"),
    ]
    for ok, message in checks:
        if not ok:
            raise PlanInputError(message)


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
def lateral_fractions(count: int) -> np.ndarray:
    """``count`` evenly spaced lateral fractions over LATERAL_FRACTION_RANGE; a single position is at 0."""
    if count == 1:
        return np.zeros(1)
    return np.linspace(LATERAL_FRACTION_RANGE[0], LATERAL_FRACTION_RANGE[1], count)


def serpentine_grid(columns: int, rows: int) -> list[tuple[int, int]]:
    """Grid indices (ix, iy) row by row, the direction along x alternating from row to row so that
    consecutive positions are neighbors."""
    order = []
    for iy in range(rows):
        xs = range(columns) if iy % 2 == 0 else reversed(range(columns))
        order.extend((ix, iy) for ix in xs)
    return order


def board_rotation_in_sensor(center_sensor: np.ndarray, tilt_deg: float, azimuth_deg: float) -> np.ndarray:
    """Rotation T -> S of a board at ``center_sensor``: facing the sensor origin, then tilted by
    ``tilt_deg`` about the in-plane axis at ``azimuth_deg`` (module docstring)."""
    ray = center_sensor / np.linalg.norm(center_sensor)
    facing = frame_from_z_axis(-ray)
    azimuth = np.radians(azimuth_deg)
    axis = np.cos(azimuth) * facing[:, 0] + np.sin(azimuth) * facing[:, 1]
    return Rotation.from_rotvec(axis * np.radians(tilt_deg)).as_matrix() @ facing


def plan_in_sensor_frame(params: PlanParameters, camera: PinholeCamera, grid: tuple[int, int]
                         ) -> tuple[list[SensorPose], dict[tuple[float, float], list[int]]]:
    """The board poses of the plan in the sensor frame, in plan order, and the counts
    {(standoff, tilt): [planned, dropped]} (a pose is dropped when a corner is too near the image edge)."""
    poses: list[SensorPose] = []
    counts: dict[tuple[float, float], list[int]] = {}
    columns, rows = grid
    fractions_x, fractions_y = lateral_fractions(columns), lateral_fractions(rows)
    for standoff in params.standoffs_mm:
        half_x, half_y = half_field_at_depth_mm(camera, standoff)
        for tilt in params.tilts_deg:
            counts[(standoff, tilt)] = [0, 0]
        for ix, iy in serpentine_grid(columns, rows):
            center = np.array([fractions_x[ix] * params.lateral_fill * half_x,
                               fractions_y[iy] * params.lateral_fill * half_y, standoff])
            for tilt in params.tilts_deg:
                azimuths = (UNTILTED_AZIMUTH_DEG,) if tilt == 0 else params.azimuths_deg
                for azimuth in azimuths:
                    counts[(standoff, tilt)][0] += 1
                    rotation = board_rotation_in_sensor(center, tilt, azimuth)
                    if not board_corners_in_view(camera, center, rotation, params.board_half_size_mm,
                                                 params.edge_margin_px):
                        counts[(standoff, tilt)][1] += 1
                        continue
                    pose_id = BOARD_ID_FORMAT.format(standoff=standoff, tilt=tilt, azimuth=azimuth, ix=ix, iy=iy)
                    poses.append(SensorPose(pose_id, standoff, tilt, azimuth, center, rotation))
    return poses, counts


def choose_lateral_grid(params: PlanParameters, camera: PinholeCamera) -> tuple[int, int]:
    """The lateral grid (columns, rows). Without ``target_pose_count`` it is the requested one.

    With it, the grids whose column and row counts are at most MAX_LATERAL_GRID_SIZE and differ by at
    most MAX_GRID_ASPECT_DIFFERENCE are planned in order of increasing number of positions, until one
    reaches the target; of the grids tried, the one whose pose count is closest to the target is taken
    (the smaller grid on a tie). Growing the grid only adds inner positions (the outermost ones stay
    at plus and minus the fill), so the pose count grows with the grid and the search can stop early;
    it is not proportional to the number of positions because positions near the field edge lose poses
    to the edge margin."""
    if params.target_pose_count is None:
        return params.lateral_positions
    candidates = sorted(
        (grid for grid in itertools.product(range(1, MAX_LATERAL_GRID_SIZE + 1), repeat=2)
         if abs(grid[0] - grid[1]) <= MAX_GRID_ASPECT_DIFFERENCE),
        key=lambda grid: (grid[0] * grid[1], grid[0] + grid[1]))
    best_grid, best_distance = candidates[0], None
    for grid in candidates:
        count = len(plan_in_sensor_frame(params, camera, grid)[0])
        distance = abs(count - params.target_pose_count)
        if best_distance is None or distance < best_distance:
            best_grid, best_distance = grid, distance
        if count >= params.target_pose_count:
            break
    return best_grid


def approach_pose_in_base(target_to_base: RigidTransform, retreat_mm: float, rotation_deg: float) -> RigidTransform:
    """The approach pose A = T o D (module docstring): the target pose T retreated by ``retreat_mm``
    along its own -z and rotated by ``rotation_deg`` about its own x axis. D is a displacement in the
    tool frame, D(x) = Rx(rotation) x + (0, 0, -retreat), and ``compose`` applies D first."""
    displacement = RigidTransform(Rotation.from_rotvec(np.radians(rotation_deg) * BOARD_X_AXIS).as_matrix(),
                                  -retreat_mm * BOARD_Z_AXIS)
    return target_to_base.compose(displacement)


def tag_holdout(poses: list[SensorPose], fraction: float, seed: int) -> None:
    """Tag a random subset (``fraction`` of all poses, rounded) as held out."""
    count = int(round(fraction * len(poses)))
    chosen = np.random.default_rng(seed).choice(len(poses), size=count, replace=False) if poses else []
    for index in chosen:
        poses[int(index)].holdout = True


# ---------------------------------------------------------------------------
# Quality of the plan
# ---------------------------------------------------------------------------
@dataclass
class PlanQuality:
    """Spreads and predicted errors of a set of planned poses."""

    pose_count: int
    normal_spread: float
    similarity_spread: float
    predicted_rotation_rms_deg: float
    predicted_translation_rms_mm: float


def plan_quality(poses: list[SensorPose], sensor_to_base: RigidTransform, params: PlanParameters) -> PlanQuality:
    """Normal spread, similarity spread and predicted registration error of the given poses.

    The normals are the planned board normals in the BASE frame (the quantity the registration's
    conditioning is judged on). The similarity spread is judged on the matrix [m_k, -d_k / max d]
    with m_k the base-frame normal and d_k the planned sensor offset, the distance from the sensor
    origin to the board's plane (the same matrix as the solver's similarity gate). The prediction is
    ``expected_registration_error`` with the assumed noise of the parameters."""
    if not poses:
        return PlanQuality(0, 0, 0, float("inf"), float("inf"))
    base_normals, sensor_offsets = [], []
    for pose in poses:
        board_to_sensor = RigidTransform(pose.board_to_sensor, pose.center_sensor)
        # The board plane carried into the sensor frame; its normal faces the sensor, so offset > 0.
        sensor_offsets.append(board_plane_in_base(board_to_sensor).offset)
        base_normals.append(sensor_to_base.rotation @ pose.board_to_sensor[:, 2])
    base_normals, sensor_offsets = np.array(base_normals), np.array(sensor_offsets)
    normal_spread = spread(base_normals)
    similarity_spread = spread(np.column_stack([base_normals, -sensor_offsets / np.max(np.abs(sensor_offsets))]))
    rotation_rms, translation_rms = expected_registration_error(
        normal_spread, len(poses), params.noise_normal_deg, params.noise_offset_mm)
    return PlanQuality(len(poses), normal_spread, similarity_spread, rotation_rms, translation_rms)


def largest_incidence_degrees(poses: list[SensorPose]) -> float:
    """Largest angle between a board's normal and the direction from its center to the sensor origin."""
    angles = []
    for pose in poses:
        to_sensor = -pose.center_sensor / np.linalg.norm(pose.center_sensor)
        cosine = float(np.clip(pose.board_to_sensor[:, 2] @ to_sensor, -1, 1))
        angles.append(np.degrees(np.arccos(cosine)))
    return float(max(angles)) if angles else 0


# ---------------------------------------------------------------------------
# Outputs
# ---------------------------------------------------------------------------
def rotation_as_quaternion_and_rotation_vector(rotation: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The unit quaternion (w, x, y, z) with w >= 0 and the rotation vector in degrees of a rotation
    matrix, computed exactly as sphcal's ``pose_row`` does for the target pose."""
    scipy_rotation = Rotation.from_matrix(rotation)
    quaternion_xyzw = scipy_rotation.as_quat()                  # SciPy orders the scalar part last
    quaternion_wxyz = np.roll(quaternion_xyzw, 1)               # move the scalar part to the front
    if quaternion_wxyz[0] < 0.0:
        quaternion_wxyz = -quaternion_wxyz                      # q and -q are the same rotation; take w >= 0
    rotation_vector_deg = np.degrees(scipy_rotation.as_rotvec())
    return quaternion_wxyz, rotation_vector_deg


def csv_row(pose: SensorPose, sensor_to_base: RigidTransform, params: PlanParameters) -> list:
    """One poses.csv row: sphcal's columns (target pose in B) and then PLAN_EXTRA_COLUMNS."""
    sphcal_pose = PlannedPose(pose_id=pose.pose_id, kind=TARGET_KIND_BOARD, radius_mm=None,
                              half_size_mm=params.board_half_size_mm, center_sensor=pose.center_sensor,
                              tool_to_sensor=pose.board_to_sensor, plane_depth_mm=pose.standoff_mm,
                              holdout=pose.holdout)
    row = pose_row(sphcal_pose, sensor_to_base)
    target = sensor_to_base.compose(RigidTransform(pose.board_to_sensor, pose.center_sensor))
    approach = approach_pose_in_base(target, params.approach_retreat_mm, params.approach_rotation_deg)
    approach_quaternion, approach_rotation_vector_deg = rotation_as_quaternion_and_rotation_vector(approach.rotation)
    return (row + [pose.standoff_mm, pose.tilt_deg, pose.azimuth_deg]
            + [float(x) for x in np.concatenate([approach.translation, approach.rotation.reshape(-1),
                                                 approach_quaternion, approach_rotation_vector_deg])])


def write_poses_csv(path: Path, poses: list[SensorPose], sensor_to_base: RigidTransform,
                    params: PlanParameters) -> None:
    """Write poses.csv (module docstring)."""
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(POSE_CSV_COLUMNS + PLAN_EXTRA_COLUMNS)
        for pose in poses:
            writer.writerow(csv_row(pose, sensor_to_base, params))


def summary_text(params: PlanParameters, camera: PinholeCamera, grid: tuple[int, int], poses: list[SensorPose],
                 counts: dict[tuple[float, float], list[int]], sensor_to_base: RigidTransform, source: str,
                 source_lines: list[str]) -> str:
    """The plan summary: counts, dropped poses, spreads, predicted errors, capture totals."""
    half_h, half_v = camera.half_angles_degrees()
    lines = ["Registration pose plan", f"Sensor-to-base transform: {source}"] + source_lines
    lines.append(f"Camera: {camera.width} x {camera.height} px, half field {half_h:.1f} x {half_v:.1f} deg")
    lines.append(f"Board half size: {params.board_half_size_mm[0]:g} x {params.board_half_size_mm[1]:g} mm; "
                 f"lateral grid {grid[0]} x {grid[1]} at fill {params.lateral_fill:g}; edge margin "
                 f"{params.edge_margin_px:g} px")
    lines.append(f"Approach pose: retreat {params.approach_retreat_mm:g} mm along the board's -z, rotation "
                 f"{params.approach_rotation_deg:g} deg about the board's x axis")
    if params.target_pose_count is not None:
        lines.append(f"Target pose count {params.target_pose_count}: the grid above gives {len(poses)}")
    lines += ["", "Poses per standoff and tilt:",
              f"  {'standoff_mm':>11} {'tilt_deg':>8} {'planned':>8} {'dropped':>8} {'kept':>6} {'held out':>9}"]
    dropped_total = 0
    for (standoff, tilt), (planned, dropped) in counts.items():
        chosen = [q for q in poses if q.standoff_mm == standoff and q.tilt_deg == tilt]
        dropped_total += dropped
        lines.append(f"  {standoff:11.0f} {tilt:8.1f} {planned:8d} {dropped:8d} {len(chosen):6d} "
                     f"{sum(q.holdout for q in chosen):9d}")
    if dropped_total:
        lines.append(f"  {dropped_total} poses were dropped because a board corner would come within "
                     f"{params.edge_margin_px:g} px of the image edge; use a smaller --lateral-fill, smaller tilts, "
                     "a smaller board or larger standoffs to keep more of them.")
    registration = RegistrationParameters()      # the minimum spreads the registration will demand
    fit_set = [q for q in poses if not q.holdout]
    for label, subset in (("all planned poses", poses), ("poses not held out", fit_set)):
        quality = plan_quality(subset, sensor_to_base, params)
        lines += ["", f"Quality of the plan, {label} ({quality.pose_count} poses):",
                  f"  normal spread      {quality.normal_spread:.4f}  (registration minimum "
                  f"{registration.minimum_normal_spread:g})",
                  f"  similarity spread  {quality.similarity_spread:.4f}  (registration minimum "
                  f"{registration.minimum_similarity_spread:g})",
                  f"  predicted rotation RMS error     {quality.predicted_rotation_rms_deg:.4f} deg",
                  f"  predicted translation RMS error  {quality.predicted_translation_rms_mm:.4f} mm",
                  f"  (assumed noise: normals {params.noise_normal_deg:g} deg, offsets {params.noise_offset_mm:g} mm; "
                  "an order of magnitude, good to a factor of about two)"]
    lines += ["", f"Largest incidence angle (board normal against the line of sight): "
                  f"{largest_incidence_degrees(poses):.1f} deg"]
    pose_visits = SUB_PROCEDURE_COUNT * len(poses)
    captures = pose_visits * params.frames_per_pose
    lines.append(f"Total: {len(poses)} poses, {sum(q.holdout for q in poses)} tagged held out (seed {params.seed}).")
    lines.append(f"Both sub-procedures together: {SUB_PROCEDURE_COUNT} x {len(poses)} poses = {pose_visits} pose "
                 f"visits; x {params.frames_per_pose} frames = {captures} capture files.")
    if len(poses) > POSE_COUNT_WARN:
        lines.append(warning_text_over_budget(len(poses)))
    return "\n".join(lines) + "\n"


def warning_text_over_budget(pose_count: int) -> str:
    """The over-budget warning line of the summary."""
    return (f"WARNING: {pose_count} poses is above {POSE_COUNT_WARN}; run about 200 poses first and plan more "
            "only if the residual analysis of that run calls for it.")


def corners_sensor(pose: SensorPose, half_size_mm: tuple[float, float]) -> np.ndarray:
    """The four board corners in the sensor frame, (4, 3), in drawing order."""
    return np.array([pose.center_sensor + pose.board_to_sensor @ np.array(
        [sx * half_size_mm[0], sy * half_size_mm[1], 0]) for sx, sy in BOARD_CORNER_SIGNS])


def write_plan_figure(path: Path, params: PlanParameters, camera: PinholeCamera, poses: list[SensorPose]) -> None:
    """Side view (x versus z) and front view (x versus y, y down as in the image) of the planned board
    centers and outlines in the sensor frame, with the field of view. One hue per standoff; held-out
    poses have dashed outlines and ringed centers. Imports matplotlib here, so the rest of the tool
    works without it (the caller handles ModuleNotFoundError)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    figure, (side, front) = plt.subplots(1, 2, figsize=PLOT_FIGURE_SIZE_IN)
    nearest, farthest = min(params.standoffs_mm), max(params.standoffs_mm)
    near_x, near_y = half_field_at_depth_mm(camera, nearest)
    far_x, far_y = half_field_at_depth_mm(camera, farthest)
    frustum = dict(color=PLOT_FRUSTUM_COLOR, linewidth=PLOT_FRUSTUM_LINE_WIDTH)
    # Side view: the two field-of-view edges in the x-z plane, from the sensor out to the farthest standoff.
    for sign in (-1, 1):
        side.plot([0, sign * far_x], [0, farthest], **frustum)
    # Front view: the image footprint at the nearest and the farthest standoff, with the corner edges between.
    for half_x, half_y in ((near_x, near_y), (far_x, far_y)):
        front.plot([-half_x, half_x, half_x, -half_x, -half_x], [-half_y, -half_y, half_y, half_y, -half_y], **frustum)
    for sx, sy in BOARD_CORNER_SIGNS:
        front.plot([sx * near_x, sx * far_x], [sy * near_y, sy * far_y], linestyle=":", **frustum)
    color_of = {s: PLOT_STANDOFF_COLORS[i % len(PLOT_STANDOFF_COLORS)] for i, s in enumerate(sorted(params.standoffs_mm))}
    # Far boards first, so that the near ones (which appear larger in the front view) are drawn on top.
    for pose in sorted(poses, key=lambda q: -q.standoff_mm):
        color = color_of[pose.standoff_mm]
        corners = corners_sensor(pose, params.board_half_size_mm)
        closed = np.vstack([corners, corners[:1]])
        style = PLOT_HOLDOUT_OUTLINE_STYLE if pose.holdout else PLOT_KEPT_OUTLINE_STYLE
        for axes, columns in ((side, (0, 2)), (front, (0, 1))):
            axes.plot(closed[:, columns[0]], closed[:, columns[1]], color=color, linestyle=style,
                      linewidth=PLOT_OUTLINE_LINE_WIDTH, alpha=PLOT_OUTLINE_ALPHA)
            axes.scatter([pose.center_sensor[columns[0]]], [pose.center_sensor[columns[1]]], s=PLOT_MARKER_SIZE,
                         c=color, edgecolors=PLOT_HOLDOUT_EDGE_COLOR if pose.holdout else color,
                         linewidths=PLOT_HOLDOUT_LINE_WIDTH if pose.holdout else 0, zorder=3)
    side.set(xlabel="sensor x (mm)", ylabel="sensor z, depth (mm)", title="Side view (x versus z)")
    front.set(xlabel="sensor x (mm)", ylabel="sensor y (mm, down)", title="Front view (x versus y)")
    front.invert_yaxis()
    for axes in (side, front):
        axes.set_aspect("equal", adjustable="datalim")
        axes.grid(True, alpha=PLOT_GRID_ALPHA)
    handles = [Line2D([0], [0], color=color_of[s], linewidth=PLOT_OUTLINE_LINE_WIDTH,
                      label=f"standoff {s:g} mm") for s in sorted(color_of)]
    handles.append(Line2D([0], [0], color=PLOT_HOLDOUT_EDGE_COLOR, linestyle=PLOT_HOLDOUT_OUTLINE_STYLE,
                          linewidth=PLOT_OUTLINE_LINE_WIDTH, label="held out (dashed)"))
    side.legend(handles=handles, loc="lower right", fontsize="small")
    figure.tight_layout()
    figure.savefig(path, dpi=PLOT_DPI)
    plt.close(figure)


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    d = PlanParameters()
    parser = argparse.ArgumentParser(
        description="Plan the robot poses of a registration session: board poses at several standoffs, lateral "
                    "positions, tilts and azimuths, each with an approach pose for the minimizing-backlash "
                    "sub-procedure. Writes poses.csv, plan_summary.txt and plan.png.")
    parser.add_argument("--sensor-in-base", required=True, type=Path, metavar="PATH",
                        help="JSON with {\"matrix\": [16 floats, row-major 4x4 sensor-to-base]} (the bootstrap tool "
                             "writes this) or sphcal's {\"bootstrap\": [{\"pose_id\", \"base_xyz\", \"sensor_xyz\"}, ...]}")
    parser.add_argument("--camera", type=Path, metavar="PATH",
                        help=".mc capture file whose header gives fx, fy, cx, cy and whose array gives the image size")
    parser.add_argument("--fov-deg", type=float, nargs=2, metavar=("H", "V"),
                        help="full horizontal and vertical field of view in degrees (with --image-size)")
    parser.add_argument("--image-size", type=int, nargs=2, metavar=("W", "H"), help="image width and height in pixels")
    parser.add_argument("--board-half-size-mm", type=float, nargs=2, default=d.board_half_size_mm,
                        metavar=("HALF_WIDTH", "HALF_HEIGHT"), help="half the long edge and half the short edge (default %(default)s)")
    parser.add_argument("--standoffs-mm", type=float, nargs="+", default=d.standoffs_mm,
                        help="sensor-z depths of the board centers (default %(default)s)")
    parser.add_argument("--tilts-deg", type=float, nargs="+", default=d.tilts_deg,
                        help="board tilts away from facing the sensor; 0 is planned once per position (default %(default)s)")
    parser.add_argument("--azimuths-deg", type=float, nargs="+", default=d.azimuths_deg,
                        help="azimuths of the tilt axis in the board plane: 0 tilts the top edge toward the sensor, 90 the "
                             "right edge, 180 the bottom edge, 270 the left edge (default %(default)s)")
    parser.add_argument("--lateral-fill", type=float, default=d.lateral_fill,
                        help="fraction of the half field covered by board centers at each standoff (default %(default)s)")
    parser.add_argument("--lateral-positions", type=int, nargs=2, default=d.lateral_positions, metavar=("NX", "NY"),
                        help="board positions across the field: columns and rows (default %(default)s)")
    parser.add_argument("--edge-margin-px", type=float, default=d.edge_margin_px,
                        help="board corners must project this far inside the image (default %(default)s)")
    parser.add_argument("--approach-retreat-mm", type=float, default=d.approach_retreat_mm,
                        help="retreat of the approach pose along the board's own -z (default %(default)s)")
    parser.add_argument("--approach-rotation-deg", type=float, default=d.approach_rotation_deg,
                        help="rotation of the approach pose about the board's own x axis (default %(default)s)")
    parser.add_argument("--holdout-fraction", type=float, default=d.holdout_fraction,
                        help="fraction of the poses tagged as held out (default %(default)s)")
    parser.add_argument("--seed", type=int, default=d.seed, help="seed of the random held-out subset")
    parser.add_argument("--target-pose-count", type=int, default=d.target_pose_count, metavar="N",
                        help="choose the lateral grid so that the plan has about N poses (overrides --lateral-positions)")
    parser.add_argument("--noise-normal-deg", type=float, default=d.noise_normal_deg,
                        help="assumed normal noise for the error prediction (default %(default)s)")
    parser.add_argument("--noise-offset-mm", type=float, default=d.noise_offset_mm,
                        help="assumed offset noise for the error prediction (default %(default)s)")
    parser.add_argument("--frames-per-pose", type=int, default=d.frames_per_pose,
                        help="frames captured per pose, for the capture counts (default %(default)s)")
    parser.add_argument("--out", required=True, type=Path, metavar="DIR", help="output directory")
    return parser


def parameters_from_arguments(args: argparse.Namespace) -> PlanParameters:
    """The PlanParameters of the parsed command line."""
    return PlanParameters(
        board_half_size_mm=tuple(args.board_half_size_mm), standoffs_mm=tuple(args.standoffs_mm),
        tilts_deg=tuple(args.tilts_deg), azimuths_deg=tuple(args.azimuths_deg), lateral_fill=args.lateral_fill,
        lateral_positions=tuple(args.lateral_positions), edge_margin_px=args.edge_margin_px,
        approach_retreat_mm=args.approach_retreat_mm, approach_rotation_deg=args.approach_rotation_deg,
        holdout_fraction=args.holdout_fraction, seed=args.seed, target_pose_count=args.target_pose_count,
        noise_normal_deg=args.noise_normal_deg, noise_offset_mm=args.noise_offset_mm,
        frames_per_pose=args.frames_per_pose)


def main(argv: list[str] | None = None) -> int:
    """Run the tool; returns the exit code (0 plan written, 2 input must be fixed)."""
    args = build_parser().parse_args(argv)
    try:
        params = parameters_from_arguments(args)
        validate_parameters(params)
        sensor_to_base, source, source_lines = load_rough_transform(args.sensor_in_base)
        camera = camera_from_arguments(args)
        grid = choose_lateral_grid(params, camera)
        poses, counts = plan_in_sensor_frame(params, camera, grid)
        if not poses:
            raise PlanInputError(
                f"no board pose of the plan keeps its four corners {params.edge_margin_px:g} px inside the image; "
                "use a smaller --lateral-fill, smaller --tilts-deg, a smaller --board-half-size-mm or larger "
                "--standoffs-mm")
        ids = [pose.pose_id for pose in poses]
        if len(set(ids)) != len(ids):
            raise PlanInputError("two planned poses got the same pose id (standoffs or angles that round to the same "
                                 "whole millimeter or degree); space them further apart")
    except PlanInputError as error:
        print_error(str(error))
        return EXIT_INPUT_ERROR
    tag_holdout(poses, params.holdout_fraction, params.seed)
    args.out.mkdir(parents=True, exist_ok=True)
    write_poses_csv(args.out / PLAN_CSV_NAME, poses, sensor_to_base, params)
    text = summary_text(params, camera, grid, poses, counts, sensor_to_base, source, source_lines)
    (args.out / PLAN_SUMMARY_NAME).write_text(text, encoding="utf-8")
    print(text, end="")
    try:
        write_plan_figure(args.out / PLAN_FIGURE_NAME, params, camera, poses)
    except ModuleNotFoundError as error:
        # The picture is a convenience; the plan itself is complete without it.
        print_warning(f"{PLAN_FIGURE_NAME} not written because the plotting package is missing ({error}); "
                      "install matplotlib (pip install matplotlib) to get the picture.")
        print(f"Wrote {args.out / PLAN_CSV_NAME} and {args.out / PLAN_SUMMARY_NAME}")
        return EXIT_OK
    print(f"Wrote {args.out / PLAN_CSV_NAME}, {args.out / PLAN_SUMMARY_NAME} and {args.out / PLAN_FIGURE_NAME}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
