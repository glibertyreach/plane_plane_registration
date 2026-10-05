"""
Synthetic registration session: a fixed depth sensor, a board carried by a robot, and the files a
real session would leave behind.

    python3 -m planereg.analysis.simulate --out DIR [--plan poses.csv] [--frames 3]
        [--sensor-to-base JSON] [--scale C] [--clutter] [--flyaways]
        [--backlash-deg X] [--backlash-mm Y] [--procedure A|B] [--seed N]

What is written (the layout of a real session)
    captures/<pose_id>_Index<nn>.mc   XYZ float32 images (sphcal ``write_matcloud``); the header holds the
                                      pinhole camera and ``robotPose`` = the REPORTED board-to-base pose
    manifest.json                     sphcal manifest, REPORTED poses
    pose_log.csv                      the technician's pose log (sphcal ``make_manifest`` format),
                                      REPORTED poses, rotation_type matrix
    truth.json                        the true sensor-to-base transform, scale, every pose reported and
                                      true, the backlash signs, all generator parameters

Reported pose versus true pose. The robot reports where it believes the board is; the board is somewhere
else by the backlash emulation (below). Everything a technician would have (headers, manifest, pose log)
carries the reported pose; the rendering uses the true pose. A registration of the session therefore sees
exactly the inconsistency a real backlash would produce.

Frames. S sensor, B robot base, T board tool frame (origin at the center of the board's front face, z toward
the sensor, x along the long edge). The true sensor-to-base transform is X_rigid (rotation R, translation s);
a reported or true board pose is T -> B. The board in the sensor frame is X_rigid^-1 o (T -> B).

Plans. With ``--plan`` the poses come from a ``poses.csv`` (columns pose_id, half_width_mm, half_height_mm,
base_x_mm..base_z_mm, r00..r22, the tool-to-base rotation, row-major; any further columns, such as the
approach pose of the capture tools' plan, are tolerated and ignored). Those poses are in the base frame and
were planned for some sensor placement: give that placement as ``--sensor-to-base``, otherwise a random one is
drawn and boards may fall out of view (a WARNING says how many). Without ``--plan`` a self-contained default
plan is generated in the sensor frame (a grid of standoffs, tilts, azimuths and lateral positions,
:class:`SimulationParameters`) and carried to the base frame with the true transform, so every board is in view.

Rendering (per frame). The board comes from sphcal's ``render_board_frame`` with its own noise model, a zero
error field by default (``--error-field default`` switches on sphcal's bowl-and-slope field). With ``--clutter``
two planes are added by ray-plane intersection with the same pinhole camera, the nearest hit per pixel winning
(the board wins where it is in front): a wall ``wall_distance_behind_mm`` behind the board center, facing the
sensor and filling the field, and a table plane below the board that ends under it (so that, as in the
design's premise, the closest large plane is the board). The clutter gets the same sensor noise law as
the board, sigma_z = k z^2 cos(incidence)^-m drawn per block of ``effective_block_px`` pixels, and the same
logistic no-read at grazing incidence. With ``--flyaways`` a fraction of the pixels within a few pixels of the
board silhouette (the board mask minus its erosion) are moved along their rays by a factor drawn uniformly from
a range around 1: the mixed pixels of a real edge.

Scale. ``--scale C`` makes the sensor report lengths in a unit 1 / C times the base unit, so that the true
sensor-to-base map is X = [C R, s; 0 1] and a similarity registration must recover C. It is implemented as a
factor 1 / C on the rendered XYZ.

Backlash emulation (a stand-in for testing the comparison, not a model of any robot). The true pose is the
reported pose with its orientation rotated by ``sign * backlash_deg`` about a fixed base-frame axis and its
position moved by ``sign * backlash_mm`` along a fixed base-frame direction. ``sign`` is random (+1 or -1) per
pose in procedure A, where the robot arrives at each pose from a different side, and constant (+1) in
procedure B, where every pose is approached the same way.

Units: millimeters, degrees at the interfaces; image arrays (height, width[, channels]); pixel (u, v) =
(column, row).
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation
from scipy.special import expit

from sphcal.cli.plan_poses import (BOARD_ID_FORMAT, PlanInputError, board_corners_in_view, frame_from_z_axis,
                                   half_field_at_depth_mm, load_sensor_in_base)
from sphcal.geometry.camera import PinholeCamera
from sphcal.geometry.transforms import RigidTransform
from sphcal.io.capture_set import XYZ_CHANNEL_NAME
from sphcal.io.matcloud import write_matcloud
from sphcal.io.poses import CaptureRecord, TARGET_KIND_BOARD, write_manifest_json
from sphcal.simulate.synthetic import (FILE_NAME_FORMAT, HEADER_POSE_KEY, MANIFEST_FILE_NAME, MINIMUM_NOISE_COSINE,
                                       PLANE_PARALLEL_TOLERANCE, SYNTHETIC_CAMERA_NAME, SYNTHETIC_HEADER_VERSION,
                                       TRUTH_FILE_NAME, SyntheticSensorParameters, default_injected_error_field_for_camera,
                                       render_board_frame)

# ---------------------------------------------------------------------------
# Constants: files, procedures, exit codes
# ---------------------------------------------------------------------------
EXIT_OK = 0
EXIT_INPUT_ERROR = 2
"""Exit codes: the session was written; an input must be fixed."""

CAPTURE_DIRECTORY_NAME = "captures"
"""Sub-directory of the output directory that holds the .mc files."""
POSE_LOG_FILE_NAME = "pose_log.csv"
"""Name of the technician-format pose log."""

PROCEDURE_A = "A"
PROCEDURE_B = "B"
PROCEDURES = (PROCEDURE_A, PROCEDURE_B)
"""Sub-procedure A ignores backlash (random approach side per pose); B minimizes it (constant approach side)."""
PROCEDURE_STREAM_INDEX = {PROCEDURE_A: 0, PROCEDURE_B: 1}
"""Index mixed into the seed of the per-procedure random streams, so that A and B of the same seed share the
sensor placement and the plan but not the noise or the backlash signs."""
CONSTANT_BACKLASH_SIGN = 1.0
"""The sign of the backlash emulation at every pose of procedure B."""
BACKLASH_SIGNS = (-1.0, 1.0)
"""The signs drawn from, per pose, in procedure A."""

ERROR_FIELD_ZERO = "zero"
ERROR_FIELD_DEFAULT = "default"
ERROR_FIELDS = (ERROR_FIELD_ZERO, ERROR_FIELD_DEFAULT)
"""Injected range error: none, or sphcal's default bowl-and-slope field."""

ROTATION_TYPE_MATRIX = "matrix"
"""rotation_type of the pose log rows: r1..r9 is the row-major rotation matrix."""
POSE_LOG_COLUMNS = (("pose_id", "kind", "radius_mm", "half_width_mm", "half_height_mm", "x_mm", "y_mm", "z_mm",
                     "rotation_type") + tuple(f"r{index}" for index in range(1, 10)))
"""Columns of the technician's pose log (sphcal make_manifest)."""

PLAN_POSE_ID_COLUMN = "pose_id"
PLAN_KIND_COLUMN = "kind"
PLAN_HALF_WIDTH_COLUMN = "half_width_mm"
PLAN_HALF_HEIGHT_COLUMN = "half_height_mm"
PLAN_POSITION_COLUMNS = ("base_x_mm", "base_y_mm", "base_z_mm")
PLAN_ROTATION_COLUMNS = tuple(f"r{row}{col}" for row in range(3) for col in range(3))
"""The columns of a plan's poses.csv this tool reads (the target pose in the base frame)."""

HOMOGENEOUS_ENTRIES = 16
"""Entries of a flattened 4 x 4 pose matrix."""
RANDOM_STREAM_COUNT = 3
"""Independent random streams per session: sensor placement, backlash signs, rendering noise."""
SENSOR_STREAM, BACKLASH_STREAM, RENDER_STREAM = range(RANDOM_STREAM_COUNT)
"""Roles of the streams."""
IMAGE_AXES = 2
"""Dimensions of an image (rows, columns): the dimension argument of the binary structuring element."""
CROSS_CONNECTIVITY = 1
"""Connectivity of the erosion that defines the board silhouette ring (1 = the four edge neighbors)."""
SENSOR_Y_AXIS = np.array([0.0, 1.0, 0.0])
"""The sensor y axis: the direction of increasing image row, "down" in the image."""
SENSOR_Z_AXIS = np.array([0.0, 0.0, 1.0])
"""The sensor z axis: the optical axis, pointing into the scene."""
NO_EDGE_MARGIN_PX = 0.0
"""Margin of the in-view test of a board after the backlash emulation: it only has to be inside the image."""
NO_DEPTH_LIMIT_MM = 0.0
"""A clutter plane with this minimum depth extends all the way to the sensor."""


@dataclass(frozen=True)
class SimulationParameters:
    """Every number of the simulation; the command-line defaults come from here."""

    # Camera
    image_width_px: int = 320
    image_height_px: int = 240
    horizontal_fov_deg: float = 60.0
    """Full horizontal field of view; the pixels are square and the principal point is the image center."""
    effective_block_px: int = 4
    """Independent depth values per block x block pixels (sphcal's value for the real sensor)."""
    frames_per_pose: int = 3

    # Board
    board_half_size_mm: tuple[float, float] = (100.0, 75.0)
    """Half width (board x) and half height (board y) of the default plan's board."""

    # Default plan (used only without --plan): standoffs x lateral positions x orientations
    standoffs_mm: tuple[float, ...] = (450.0, 600.0, 750.0)
    orientations_deg: tuple[tuple[float, float], ...] = ((0.0, 0.0), (25.0, 0.0), (25.0, 120.0), (25.0, 240.0))
    """(tilt, azimuth) pairs: the board facing the sensor, then tilted by 25 degrees about three in-plane axes
    spaced 120 degrees apart, so that the plate normals spread over a cone."""
    lateral_positions: tuple[tuple[float, float], ...] = ((-1.0, 1.0), (1.0, -1.0))
    """Board centers across the field as fractions (x, y) of the lateral extent; two opposite corners."""
    lateral_fill: float = 0.35
    """Fraction of the half field covered by board centers at each standoff."""
    edge_margin_px: float = 10.0
    """A planned board whose corners come closer than this to the image border is dropped."""

    # Clutter
    wall_distance_behind_mm: float = 300.0
    """The wall plane lies this far behind the board center, along the ray through the center."""
    table_drop_mm: float = 200.0
    """The table plane passes this far below the board center (along the sensor +y axis, image down)."""
    table_elevation_deg: float = 30.0
    """Angle of the table normal out of the sensor's x-z plane toward the sensor: the table is seen from
    above, receding upward in the image."""
    table_near_edge_offset_mm: float = 0.0
    """The table ends under the board: it exists only at camera-z depths at least this much beyond the board
    center's depth. A table that reached toward the sensor past the board would be the closest large plane."""

    # Fly-aways
    flyaway_fraction: float = 0.3
    """Fraction of the silhouette-ring pixels that are displaced."""
    flyaway_band_px: int = 2
    """Width of the silhouette ring: the board mask minus its erosion by this many pixels."""
    flyaway_factor_range: tuple[float, float] = (0.9, 1.1)
    """A displaced pixel moves along its ray by a factor drawn uniformly from this range."""

    # Backlash emulation
    backlash_deg: float = 0.0
    backlash_mm: float = 0.0
    backlash_axis: tuple[float, float, float] = (1.0, 2.0, 2.0)
    """Base-frame rotation axis of the orientation backlash (normalized when used)."""
    backlash_direction: tuple[float, float, float] = (2.0, -1.0, 2.0)
    """Base-frame direction of the position backlash (normalized when used)."""

    # Session
    procedure: str = PROCEDURE_A
    scale: float = 1.0
    """True scale c of the sensor-to-base map X = [c R, s; 0 1]; the rendered XYZ is divided by it."""
    error_field: str = ERROR_FIELD_ZERO
    clutter: bool = False
    flyaways: bool = False
    seed: int = 0
    sensor_translation_range_mm: float = 1000.0
    """A random sensor placement draws each translation component uniformly from +/- this range."""


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------
@dataclass
class PlannedBoard:
    """One board pose to visit: the reported target pose T -> B and the board's half sizes."""

    pose_id: str
    half_size_mm: tuple[float, float]
    reported_pose: RigidTransform


@dataclass
class ClutterPlane:
    """A plane of the background: a point on it and its unit normal, both in the sensor frame, the normal
    pointing toward the sensor."""

    name: str
    point_mm: np.ndarray
    normal: np.ndarray
    minimum_depth_mm: float = NO_DEPTH_LIMIT_MM
    """Hits nearer to the sensor than this camera-z depth are not part of the surface (a surface that ends)."""


@dataclass
class SimulatedPose:
    """Everything the generator knows about one pose."""

    planned: PlannedBoard
    true_pose: RigidTransform
    backlash_sign: float
    board_center_sensor_mm: np.ndarray = field(default_factory=lambda: np.zeros(3))


class SimulationInputError(Exception):
    """An input the user has to fix; the message says what to change."""


# ---------------------------------------------------------------------------
# Sensor and camera
# ---------------------------------------------------------------------------
def camera_from_parameters(params: SimulationParameters) -> PinholeCamera:
    """A distortion-free pinhole camera with square pixels, the principal point at the image center and the
    given horizontal field of view."""
    focal_px = (params.image_width_px / 2) / np.tan(np.radians(params.horizontal_fov_deg) / 2)
    return PinholeCamera(params.image_width_px, params.image_height_px, focal_px, focal_px,
                         params.image_width_px / 2, params.image_height_px / 2)


def sensor_parameters(params: SimulationParameters, camera: PinholeCamera) -> SyntheticSensorParameters:
    """sphcal's indicative sensor behavior (noise law, no-read law) on the given camera."""
    return replace(SyntheticSensorParameters.vsx3000_indicative(), camera=camera,
                   effective_block_px=params.effective_block_px)


def zero_error_field(u, v, rho, s_u, s_v, curvature) -> np.ndarray:
    """No injected range error: the board is rendered exactly where it is (apart from noise)."""
    return np.zeros_like(np.asarray(rho, dtype=np.float64))


zero_error_field.description = "no injected range error"  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# Poses: sensor placement, plans
# ---------------------------------------------------------------------------
def random_sensor_to_base(rng: np.random.Generator, params: SimulationParameters) -> RigidTransform:
    """A random rigid sensor-to-base transform: a uniformly distributed rotation and a translation drawn
    uniformly from a cube. Any placement works because the default plan is generated in the sensor frame."""
    rotation = Rotation.random(random_state=rng).as_matrix()
    limit = params.sensor_translation_range_mm
    return RigidTransform(rotation, rng.uniform(-limit, limit, size=3))


def default_plan(params: SimulationParameters, camera: PinholeCamera,
                 sensor_to_base: RigidTransform) -> tuple[list[PlannedBoard], int]:
    """The self-contained plan: for every standoff, lateral position and (tilt, azimuth), a board whose
    z axis faces the sensor origin, tilted about the in-plane axis at the azimuth (azimuth 0 = board x axis),
    exactly as the capture tools' plan orients its boards. Returns the poses carried to the base frame with
    ``sensor_to_base`` and the number of poses dropped because a corner fell within the edge margin."""
    boards: list[PlannedBoard] = []
    dropped = 0
    for depth in params.standoffs_mm:
        half_x, half_y = half_field_at_depth_mm(camera, depth)
        for tilt_deg, azimuth_deg in params.orientations_deg:
            for index, (fraction_x, fraction_y) in enumerate(params.lateral_positions):
                center = np.array([fraction_x * params.lateral_fill * half_x,
                                   fraction_y * params.lateral_fill * half_y, depth])
                facing = frame_from_z_axis(-center / np.linalg.norm(center))
                tilt_axis = (np.cos(np.radians(azimuth_deg)) * facing[:, 0]
                             + np.sin(np.radians(azimuth_deg)) * facing[:, 1])
                rotation = Rotation.from_rotvec(tilt_axis * np.radians(tilt_deg)).as_matrix() @ facing
                if not board_corners_in_view(camera, center, rotation, params.board_half_size_mm,
                                             params.edge_margin_px):
                    dropped += 1
                    continue
                pose_id = BOARD_ID_FORMAT.format(depth=depth, tilt=tilt_deg, azimuth=azimuth_deg, index=index)
                board_to_sensor = RigidTransform(rotation, center)
                boards.append(PlannedBoard(pose_id, params.board_half_size_mm, sensor_to_base.compose(board_to_sensor)))
    return boards, dropped


def load_plan_csv(path: Path, default_half_size_mm: tuple[float, float]) -> tuple[list[PlannedBoard], list[str]]:
    """Read the board poses of a plan's poses.csv (target pose in the base frame). Rows of another kind than
    board are skipped and named in the returned warnings; columns this tool does not know are ignored."""
    try:
        handle = Path(path).open("r", newline="", encoding="utf-8-sig")
    except OSError as error:
        raise SimulationInputError(f"cannot read the plan {path}: {error.strerror}") from None
    with handle:
        reader = csv.DictReader(handle)
        columns = [name.strip() for name in (reader.fieldnames or [])]
        rows = [{(key or "").strip(): value for key, value in row.items()} for row in reader]
    required = (PLAN_POSE_ID_COLUMN,) + PLAN_POSITION_COLUMNS + PLAN_ROTATION_COLUMNS
    missing = [name for name in required if name not in columns]
    if missing:
        raise SimulationInputError(f"the plan {path} lacks the columns {missing}; give the poses.csv written by "
                                   "the plan tool")
    boards: list[PlannedBoard] = []
    warnings: list[str] = []
    for line_number, row in enumerate(rows, start=2):       # line 1 is the header
        pose_id = (row.get(PLAN_POSE_ID_COLUMN) or "").strip()
        kind = (row.get(PLAN_KIND_COLUMN) or TARGET_KIND_BOARD).strip().lower()
        if kind != TARGET_KIND_BOARD:
            warnings.append(f"plan line {line_number} (pose {pose_id}): kind {kind!r} is not a board; skipped")
            continue
        try:
            position = np.array([float(row[name]) for name in PLAN_POSITION_COLUMNS])
            rotation = np.array([float(row[name]) for name in PLAN_ROTATION_COLUMNS]).reshape(3, 3)
            half_width = float(row.get(PLAN_HALF_WIDTH_COLUMN) or default_half_size_mm[0])
            half_height = float(row.get(PLAN_HALF_HEIGHT_COLUMN) or default_half_size_mm[1])
        except (TypeError, ValueError):
            raise SimulationInputError(f"plan line {line_number} (pose {pose_id}): the position and rotation "
                                       "columns must be plain numbers") from None
        boards.append(PlannedBoard(pose_id, (half_width, half_height), RigidTransform(rotation, position)))
    if not boards:
        raise SimulationInputError(f"the plan {path} holds no board pose")
    return boards, warnings


def apply_backlash(reported: RigidTransform, sign: float, params: SimulationParameters) -> RigidTransform:
    """The true pose for a reported pose and a backlash sign (module docstring): the orientation rotated about
    the fixed base axis by sign * backlash_deg, the position moved along the fixed base direction by
    sign * backlash_mm."""
    axis = np.asarray(params.backlash_axis, dtype=np.float64)
    direction = np.asarray(params.backlash_direction, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    direction = direction / np.linalg.norm(direction)
    rotation_step = Rotation.from_rotvec(axis * np.radians(sign * params.backlash_deg)).as_matrix()
    return RigidTransform(rotation_step @ reported.rotation, reported.translation + sign * params.backlash_mm * direction)


def backlash_signs(count: int, params: SimulationParameters, rng: np.random.Generator) -> np.ndarray:
    """One sign per pose: random in procedure A, constant in procedure B."""
    if params.procedure == PROCEDURE_B:
        return np.full(count, CONSTANT_BACKLASH_SIGN)
    return rng.choice(np.asarray(BACKLASH_SIGNS), size=count)


# ---------------------------------------------------------------------------
# Rendering: clutter, fly-aways, one frame
# ---------------------------------------------------------------------------
def clutter_planes(board_center_sensor: np.ndarray, params: SimulationParameters) -> list[ClutterPlane]:
    """The wall and the table of a pose (sensor frame, true board position). The wall is perpendicular to the
    ray through the board center, ``wall_distance_behind_mm`` farther along it. The table passes
    ``table_drop_mm`` below the board center along the sensor y axis; its normal points up in the image and
    toward the sensor (``table_elevation_deg`` out of the x-z plane), so the table recedes toward the top of
    the image."""
    ray = board_center_sensor / np.linalg.norm(board_center_sensor)
    wall = ClutterPlane("wall", board_center_sensor + params.wall_distance_behind_mm * ray, -ray)
    elevation = np.radians(params.table_elevation_deg)
    table_normal = -np.cos(elevation) * SENSOR_Y_AXIS - np.sin(elevation) * SENSOR_Z_AXIS
    table = ClutterPlane("table", board_center_sensor + params.table_drop_mm * SENSOR_Y_AXIS, table_normal,
                         board_center_sensor[2] + params.table_near_edge_offset_mm)
    return [wall, table]


def block_draws(shape: tuple[int, int], block_px: int, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """Two (H, W) images of independent draws per block of block_px x block_px pixels, constant over each
    block: standard normal values (noise) and uniform [0, 1) values (no-read decisions). This is the
    block-wise independence of the sensor's effective resolution, without the interpolation sphcal does for
    the board."""
    rows = -(-shape[0] // block_px)
    columns = -(-shape[1] // block_px)
    normal = rng.standard_normal((rows, columns))
    uniform = rng.random((rows, columns))

    def expand(image: np.ndarray) -> np.ndarray:
        return np.repeat(np.repeat(image, block_px, axis=0), block_px, axis=1)[:shape[0], :shape[1]]

    return expand(normal), expand(uniform)


def render_clutter(sensor: SyntheticSensorParameters, planes: list[ClutterPlane],
                   rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """The background seen by the sensor: for every pixel the nearest hit among the planes (exact ray-plane
    intersection), with the sensor's noise and no-read laws. Returns (xyz (H, W, 3) float64 with zeros where
    nothing was read, range (H, W) with +inf where no plane was hit, regardless of the no-read)."""
    camera = sensor.camera
    rays = camera.ray_directions()
    nearest_range = np.full(rays.shape[:2], np.inf)
    cosine_incidence = np.ones(rays.shape[:2])
    for plane in planes:
        facing = rays @ plane.normal                          # n . d, negative when the plane faces the sensor
        hit_front = facing < -PLANE_PARALLEL_TOLERANCE
        range_to_plane = (plane.normal @ plane.point_mm) / np.where(hit_front, facing, -1)
        beyond_edge = range_to_plane * rays[..., 2] >= plane.minimum_depth_mm
        closer = hit_front & (range_to_plane > 0) & beyond_edge & (range_to_plane < nearest_range)
        nearest_range = np.where(closer, range_to_plane, nearest_range)
        cosine_incidence = np.where(closer, -facing, cosine_incidence)
    hit = np.isfinite(nearest_range)
    safe_range = np.where(hit, nearest_range, 0)
    ray_z = rays[..., 2]
    true_z = safe_range * ray_z
    noise_draw, read_draw = block_draws(hit.shape, sensor.effective_block_px, rng)
    sigma_z = (sensor.noise_coefficient_per_mm * true_z ** 2
               * np.maximum(cosine_incidence, MINIMUM_NOISE_COSINE) ** (-sensor.noise_incidence_exponent))
    final_z = true_z + sigma_z * noise_draw
    incidence_deg = np.degrees(np.arccos(np.clip(cosine_incidence, 0, 1)))
    read_probability = expit(-(incidence_deg - sensor.noread_onset_deg) / sensor.noread_width_deg)
    read = hit & (read_draw < read_probability) & (final_z > 0)
    final_range = np.where(read, final_z / np.where(read, ray_z, 1), 0)
    return rays * final_range[..., None], nearest_range


def apply_flyaways(xyz: np.ndarray, board_mask: np.ndarray, params: SimulationParameters,
                   rng: np.random.Generator) -> int:
    """Displace, in place, a fraction of the silhouette-ring pixels of the board along their rays. The ring is
    the board mask minus its erosion by ``flyaway_band_px``. Returns the number of displaced pixels."""
    structure = ndimage.generate_binary_structure(IMAGE_AXES, CROSS_CONNECTIVITY)
    eroded = ndimage.binary_erosion(board_mask, structure=structure, iterations=params.flyaway_band_px,
                                    border_value=1)
    ring = board_mask & ~eroded
    chosen = ring & (rng.random(ring.shape) < params.flyaway_fraction)
    count = int(chosen.sum())
    low, high = params.flyaway_factor_range
    xyz[chosen] *= rng.uniform(low, high, size=count)[:, None].astype(xyz.dtype)
    return count


def render_frame(sensor: SyntheticSensorParameters, board_to_sensor: RigidTransform,
                 half_size_mm: tuple[float, float], params: SimulationParameters, error_field,
                 rng: np.random.Generator) -> np.ndarray:
    """One XYZ frame (H, W, 3) float32 in the sensor's length unit: the true board by sphcal's renderer, the
    optional clutter behind and below it, the optional fly-aways, and the length-unit factor 1 / scale."""
    board = render_board_frame(sensor, board_to_sensor, half_size_mm, error_field, rng)
    xyz = board.xyz.astype(np.float64)
    board_wins = board.hit_mask
    if params.clutter:
        clutter_xyz, clutter_range = render_clutter(sensor, clutter_planes(board_to_sensor.translation, params), rng)
        board_range = np.where(board.hit_mask, board.true_range, np.inf)
        board_wins = board.hit_mask & (board_range < clutter_range)
        xyz = np.where(board_wins[..., None], xyz, clutter_xyz)
    xyz = xyz.astype(np.float32)
    if params.flyaways:
        # The board mask is where a board point was actually delivered (hit, read, and in front).
        apply_flyaways(xyz, board_wins & board.read_mask, params, rng)
    return (xyz / np.float32(params.scale)).astype(np.float32)


# ---------------------------------------------------------------------------
# Session
# ---------------------------------------------------------------------------
def flat_matrix(transform: RigidTransform) -> list[float]:
    """The row-major 16 floats of a transform's 4 x 4 matrix."""
    return [float(value) for value in transform.as_matrix().reshape(-1)]


def write_pose_log(path: Path, boards: list[PlannedBoard]) -> None:
    """The technician's pose log: the REPORTED pose of every board, rotation_type matrix."""
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(POSE_LOG_COLUMNS)
        for board in boards:
            pose = board.reported_pose
            writer.writerow([board.pose_id, TARGET_KIND_BOARD, "", repr(board.half_size_mm[0]),
                             repr(board.half_size_mm[1])]
                            + [repr(float(value)) for value in pose.translation]
                            + [ROTATION_TYPE_MATRIX]
                            + [repr(float(value)) for value in pose.rotation.reshape(-1)])


def simulate_session(out_dir: Path, params: SimulationParameters, plan: list[PlannedBoard] | None = None,
                     sensor_to_base: RigidTransform | None = None) -> dict:
    """Render and write a session into ``out_dir`` (module docstring) and return the truth dictionary that was
    written to truth.json. ``plan`` are boards in the base frame; without it the default plan is generated.
    ``sensor_to_base`` is the true rigid sensor-to-base transform; without it a seeded random one is drawn."""
    if params.procedure not in PROCEDURES:
        raise SimulationInputError(f"the procedure must be one of {PROCEDURES}, not {params.procedure!r}")
    if not params.scale > 0:
        raise SimulationInputError(f"the scale must be positive, not {params.scale}")
    out_path = Path(out_dir)
    captures_path = out_path / CAPTURE_DIRECTORY_NAME
    captures_path.mkdir(parents=True, exist_ok=True)

    # Random streams: the sensor placement depends on the seed only (A and B of one seed share it); the
    # backlash signs and the rendering noise also depend on the procedure.
    sensor_streams = np.random.SeedSequence(params.seed).spawn(RANDOM_STREAM_COUNT)
    procedure_streams = np.random.SeedSequence(
        [params.seed, PROCEDURE_STREAM_INDEX[params.procedure]]).spawn(RANDOM_STREAM_COUNT)
    sensor_rng = np.random.default_rng(sensor_streams[SENSOR_STREAM])
    backlash_rng = np.random.default_rng(procedure_streams[BACKLASH_STREAM])
    render_rng = np.random.default_rng(procedure_streams[RENDER_STREAM])
    camera = camera_from_parameters(params)
    sensor = sensor_parameters(params, camera)
    true_x = sensor_to_base if sensor_to_base is not None else random_sensor_to_base(sensor_rng, params)
    messages: list[str] = []
    if plan is None:
        boards, dropped = default_plan(params, camera, true_x)
        if dropped:
            messages.append(f"WARNING: {dropped} planned board(s) fell too close to the image border and were dropped")
    else:
        boards = plan
    if not boards:
        raise SimulationInputError("the plan holds no pose inside the field of view")

    signs = backlash_signs(len(boards), params, backlash_rng)
    error_field = (default_injected_error_field_for_camera(camera) if params.error_field == ERROR_FIELD_DEFAULT
                   else zero_error_field)
    poses: list[SimulatedPose] = []
    records: list[CaptureRecord] = []
    out_of_view = 0
    for board, sign in zip(boards, signs):
        true_pose = apply_backlash(board.reported_pose, float(sign), params)
        board_to_sensor = true_x.inverse().compose(true_pose)
        if not board_corners_in_view(camera, board_to_sensor.translation, board_to_sensor.rotation,
                                     board.half_size_mm, NO_EDGE_MARGIN_PX):
            out_of_view += 1
        poses.append(SimulatedPose(board, true_pose, float(sign), board_to_sensor.translation.copy()))
        for frame in range(params.frames_per_pose):
            xyz = render_frame(sensor, board_to_sensor, board.half_size_mm, params, error_field, render_rng)
            header = {"fx": camera.focal_x_px, "fy": camera.focal_y_px,
                      "cx": camera.principal_x_px, "cy": camera.principal_y_px,
                      "h": camera.width, "v": camera.height,
                      "cameraName": SYNTHETIC_CAMERA_NAME, "version": SYNTHETIC_HEADER_VERSION,
                      HEADER_POSE_KEY: flat_matrix(board.reported_pose)}
            file_path = captures_path / FILE_NAME_FORMAT.format(pose_id=board.pose_id, frame=frame)
            write_matcloud(file_path, header, {XYZ_CHANNEL_NAME: xyz})
            records.append(CaptureRecord(
                path=file_path, pose_id=board.pose_id, frame_index=frame, target_kind=TARGET_KIND_BOARD,
                sphere_radius_mm=None, board_half_size_mm=board.half_size_mm,
                target_pose_positioner=board.reported_pose, metadata={}))
    if out_of_view:
        messages.append(f"WARNING: {out_of_view} board(s) are not fully inside the image for this sensor placement; "
                        "give the --sensor-to-base the plan was made for")

    write_manifest_json(out_path / MANIFEST_FILE_NAME, records)
    write_pose_log(out_path / POSE_LOG_FILE_NAME, boards)
    rotation_c = params.scale * true_x.rotation
    truth = {
        "generator": "planereg.analysis.simulate",
        "parameters": asdict(params),
        "sensor": asdict(sensor),
        "error_field": getattr(error_field, "description", ERROR_FIELD_DEFAULT),
        "procedure": params.procedure,
        "seed": params.seed,
        "sensor_to_base": {
            "matrix": [float(value) for value in np.block([[rotation_c, true_x.translation[:, None]],
                                                          [np.zeros((1, 3)), np.ones((1, 1))]]).reshape(-1)],
            "rotation": true_x.rotation.tolist(),
            "translation_mm": true_x.translation.tolist(),
            "scale": params.scale,
        },
        "poses": [{"pose_id": pose.planned.pose_id,
                   "half_size_mm": list(pose.planned.half_size_mm),
                   "reported_pose": flat_matrix(pose.planned.reported_pose),
                   "true_pose": flat_matrix(pose.true_pose),
                   "backlash_sign": pose.backlash_sign,
                   "board_center_sensor_mm": pose.board_center_sensor_mm.tolist()} for pose in poses],
        "messages": messages,
    }
    (out_path / TRUTH_FILE_NAME).write_text(json.dumps(truth, indent=2), encoding="utf-8")
    return truth


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    defaults = SimulationParameters()
    parser = argparse.ArgumentParser(
        prog="python3 -m planereg.analysis.simulate",
        description="Write a synthetic registration session (captures, manifest, pose log, truth).")
    parser.add_argument("--out", required=True, type=Path, help="output directory (created)")
    parser.add_argument("--plan", type=Path, default=None,
                        help="poses.csv of the plan tool (target poses in the base frame); default: a built-in "
                             "grid of about 24 poses generated in the sensor frame")
    parser.add_argument("--frames", type=int, default=defaults.frames_per_pose, help="frames per pose")
    parser.add_argument("--sensor-to-base", type=Path, default=None,
                        help="JSON {\"matrix\": [16 floats]}: the true rigid sensor-to-base transform "
                             "(default: random, seeded)")
    parser.add_argument("--scale", type=float, default=defaults.scale,
                        help="true scale of the sensor-to-base map (the sensor's length unit is 1/scale of the base's)")
    parser.add_argument("--clutter", action="store_true", help="add a wall behind and a table below the board")
    parser.add_argument("--flyaways", action="store_true", help="displace pixels on the board silhouette")
    parser.add_argument("--backlash-deg", type=float, default=defaults.backlash_deg,
                        help="orientation difference between reported and true pose, degrees")
    parser.add_argument("--backlash-mm", type=float, default=defaults.backlash_mm,
                        help="position difference between reported and true pose, millimeters")
    parser.add_argument("--procedure", choices=PROCEDURES, default=defaults.procedure,
                        help="A: random backlash sign per pose; B: constant sign")
    parser.add_argument("--seed", type=int, default=defaults.seed)
    parser.add_argument("--error-field", choices=ERROR_FIELDS, default=defaults.error_field,
                        help="injected range error: zero, or sphcal's bowl-and-slope field")
    parser.add_argument("--image-size", type=int, nargs=2, metavar=("W", "H"),
                        default=(defaults.image_width_px, defaults.image_height_px), help="image size in pixels")
    parser.add_argument("--fov-deg", type=float, default=defaults.horizontal_fov_deg,
                        help="full horizontal field of view in degrees")
    parser.add_argument("--board-half-size-mm", type=float, nargs=2, metavar=("HW", "HH"),
                        default=defaults.board_half_size_mm, help="board half width and half height")
    return parser


def parameters_from_arguments(args: argparse.Namespace) -> SimulationParameters:
    return SimulationParameters(
        image_width_px=int(args.image_size[0]), image_height_px=int(args.image_size[1]),
        horizontal_fov_deg=args.fov_deg, frames_per_pose=args.frames,
        board_half_size_mm=(float(args.board_half_size_mm[0]), float(args.board_half_size_mm[1])),
        backlash_deg=args.backlash_deg, backlash_mm=args.backlash_mm, procedure=args.procedure,
        scale=args.scale, error_field=args.error_field, clutter=args.clutter, flyaways=args.flyaways,
        seed=args.seed)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    params = parameters_from_arguments(args)
    try:
        sensor_to_base = None
        if args.sensor_to_base is not None:
            sensor_to_base, _, _ = load_sensor_in_base(args.sensor_to_base, residual_warn_mm=np.inf)
        plan, plan_warnings = (None, []) if args.plan is None else load_plan_csv(args.plan, params.board_half_size_mm)
        for warning in plan_warnings:
            print(f"WARNING: {warning}")
        truth = simulate_session(args.out, params, plan, sensor_to_base)
    except (PlanInputError, SimulationInputError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    for message in truth["messages"]:
        print(message)
    print(f"wrote {len(truth['poses'])} poses x {params.frames_per_pose} frames, procedure {params.procedure}, "
          f"to {args.out}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
