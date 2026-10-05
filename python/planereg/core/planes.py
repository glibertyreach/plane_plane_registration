"""
Plane algebra in the conventions of the registration specification and REVIEW.md.

A plane is the set of points p with  n . p + d = 0.  When |n| = 1,  n . p + d  is the signed
distance of p from the plane, positive on the side n points to, and the origin lies on the
positive side exactly when d > 0.

Planes transform by the inverse transpose of the point map (specification Section 4): for a
rigid map  T = [R t; 0 1]  of points, the plane (n, d) becomes (R n, d - t . (R n)); the normal's
length is preserved.

Frames used throughout planereg
    S   the sensor frame (the frame of the points in a capture file)
    B   the robot base frame (the frame the controller reports poses in; "world" in the spec)
    T   the board tool frame: origin at the center of the board's front face, z out of the face
        toward the sensor, x along the long edge (stage-1 procedure, Section 4). The manifest's
        target pose is T -> B.

Orientation convention (author's decision, REVIEW.md Section 9): the board faces the sensor, so
the sensor-frame plane of the board is stored with its normal pointing toward the sensor, which
with the sensor at the origin means d > 0.

Units: millimeters.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sphcal.geometry.transforms import RigidTransform

ZERO_LENGTH_TOLERANCE = 1e-12
"""A normal shorter than this cannot be normalized."""

BOARD_FACE_NORMAL_IN_TOOL_FRAME = np.array([0.0, 0.0, 1.0])
"""The board's front face normal in the board tool frame: +z, toward the sensor."""


@dataclass(frozen=True)
class Plane:
    """A plane n . p + d = 0. ``normal`` is a length-3 array; not necessarily unit length
    until :meth:`normalized` has been applied."""

    normal: np.ndarray
    offset: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "normal", np.asarray(self.normal, dtype=np.float64).reshape(3))
        object.__setattr__(self, "offset", float(self.offset))

    @classmethod
    def from_coefficients(cls, coefficients) -> "Plane":
        """From the specification's (a, b, c, d)."""
        a, b, c, d = (float(x) for x in np.asarray(coefficients, dtype=np.float64).reshape(4))
        return cls(np.array([a, b, c]), d)

    def coefficients(self) -> np.ndarray:
        """(a, b, c, d)."""
        return np.r_[self.normal, self.offset]

    def normalized(self, origin_on_positive_side: bool = True) -> "Plane":
        """Scale to a unit normal and choose the sign so that the origin lies on the requested
        side (offset > 0 when ``origin_on_positive_side``, offset < 0 otherwise). An offset of
        exactly zero keeps its sign. Raises ValueError for a zero-length or non-finite normal."""
        length = float(np.linalg.norm(self.normal))
        if not np.isfinite(length) or length <= ZERO_LENGTH_TOLERANCE or not np.isfinite(self.offset):
            raise ValueError(f"cannot normalize the plane {self}")
        normal, offset = self.normal / length, self.offset / length
        wrong_sign = (offset < 0.0) if origin_on_positive_side else (offset > 0.0)
        if wrong_sign:
            normal, offset = -normal, -offset
        return Plane(normal, offset)

    def unit(self) -> "Plane":
        """Scale to a unit normal without touching the sign (for planes whose orientation is
        already meaningful, such as the base-frame plate plane whose normal is the flange +z)."""
        length = float(np.linalg.norm(self.normal))
        if not np.isfinite(length) or length <= ZERO_LENGTH_TOLERANCE or not np.isfinite(self.offset):
            raise ValueError(f"cannot normalize the plane {self}")
        return Plane(self.normal / length, self.offset / length)

    def signed_distance(self, points) -> np.ndarray:
        """n . p + d for points of shape (..., 3); a true distance only for a unit normal."""
        points = np.asarray(points, dtype=np.float64)
        return points @ self.normal + self.offset

    def angle_to_degrees(self, other: "Plane") -> float:
        """Angle between the two normals in degrees (both assumed unit length), computed with
        atan2 so that tiny angles keep their relative accuracy."""
        sine = float(np.linalg.norm(np.cross(self.normal, other.normal)))
        cosine = float(self.normal @ other.normal)
        return float(np.degrees(np.arctan2(sine, cosine)))


def transform_plane(point_map: RigidTransform, plane: Plane) -> Plane:
    """The plane after the points it contains are moved by ``point_map``:
    (R n, d - t . (R n)), the closed form of the inverse transpose (spec Eq. 36)."""
    new_normal = point_map.rotation @ plane.normal
    return Plane(new_normal, plane.offset - float(point_map.translation @ new_normal))


def board_plane_in_base(board_to_base: RigidTransform, target_offset_mm: float = 0.0) -> Plane:
    """The board's front-face plane in the base frame, from the reported pose of the board tool
    frame (T -> B). ``target_offset_mm`` is the distance of the face along the frame's +z axis
    from the frame's origin: 0 when the robot reports the board tool frame itself (the stage-1
    procedure's TOOL_BOARD), or the plate thickness / adapter offset D when the robot reports
    the flange frame instead (spec Eq. 3, o = (0, 0, 1, -D))."""
    plane_in_tool = Plane(BOARD_FACE_NORMAL_IN_TOOL_FRAME, -float(target_offset_mm))
    return transform_plane(board_to_base, plane_in_tool).unit()


def predicted_sensor_plane(sensor_to_base: RigidTransform, base_plane: Plane, scale: float = 1.0) -> Plane:
    """The plane the sensor should measure, given the base-frame plane (m, e) and the
    sensor-to-base map X = [scale R, s; 0 1]:  X^t (m, e) = (scale R^t m, s . m + e), scaled to
    the unit normal (R^t m, (s . m + e) / scale)  (REVIEW.md Section 4.7)."""
    rotation, translation = sensor_to_base.rotation, sensor_to_base.translation
    normal = rotation.T @ base_plane.normal
    offset = (float(translation @ base_plane.normal) + base_plane.offset) / float(scale)
    return Plane(normal, offset)
