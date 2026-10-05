"""Figure: the two residuals the software reports for every pose.

One panel, drawn in the sensor's view plane (a side view, like the approach figure).
Blue: the predicted board, i.e. where the board should be according to the logged robot pose
and the current sensor-to-robot transform. Orange: the measured board, i.e. the plane fitted to
the sensor's own points. The two nearly coincide; the differences are exaggerated here so they
can be seen:
  - the normal residual is the angle between the two board normals (reported in degrees);
  - the offset residual is the perpendicular gap between the two planes, measured at the
    predicted board's center along the predicted normal (reported in millimeters).
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, FancyArrowPatch

# ---- Colors and line styles (same as make_fig_approach.py) ----
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
GRID = "#e6e5e1"
SERIES_TARGET = "#2a78d6"     # the predicted board and its normal
SERIES_APPROACH = "#eb6834"   # the measured board, its normal and its points
SENSOR_COLOR = "#52514e"
LINE_WIDTH = 2.0              # base line width, points
BOARD_LINE_FACTOR = 2         # boards are drawn this many times thicker than LINE_WIDTH
THIN_LINE_WIDTH = 1.0         # faint rays and reference lines, points
DPI = 150
FIGURE_SIZE = (5.2, 3.6)      # inches (width, height)

# ---- Text sizes, points ----
TITLE_FONT_SIZE = 10          # caption line above the axes
LABEL_FONT_SIZE = 9           # sensor name and residual labels
LEGEND_FONT_SIZE = 9
ARROW_MUTATION_SCALE = 12     # arrow head size for the normals
DIMENSION_MUTATION_SCALE = 6  # arrow head size for the double-headed dimension line

# ---- Axes limits, drawing units ----
X_MIN, X_MAX = -2.7, 2.7
Y_MIN, Y_MAX = -0.4, 3.9

# ---- Sensor marker and field of view ----
SENSOR_POSITION = np.array([0.0, 3.6])  # sensor marker, drawing units
SENSOR_MARKER_SIZE = 10                 # square marker size, points
SENSOR_MARKER_EDGE_WIDTH = 2            # surface-colored edge around the marker, points
SENSOR_LABEL_OFFSET_X = 0.15            # gap between the marker and its "sensor" label
RAY_END_Y = 0.7                         # height at which the two field-of-view rays stop
RAY_HALF_WIDTH = 2.5                    # horizontal reach of each ray at RAY_END_Y

# ---- Boards ----
BOARD_HALF_LENGTH = 1.0                 # half the drawn board length, drawing units
BOARD_CENTER = np.array([0.1, 1.3])     # center of the predicted board
TILT_DEG = 20.0                         # tilt of the predicted board from horizontal
ANGLE_RESIDUAL_DEG = 12.0               # exaggerated rotation of the measured board (real values are tiny)
OFFSET_RESIDUAL = 0.35                  # exaggerated shift of the measured board along the predicted normal
NORMAL_ARROW_LENGTH = 1.2               # length of the drawn board normals, drawing units
PREDICTED_NORMAL_BASE_SHIFT = -0.6      # the predicted normal starts this far along its board from the
                                        # center (to the left), clear of the dimension line at the center
MEASURED_NORMAL_BASE_SHIFT = 0.6        # the measured normal starts this far along its board (to the
                                        # right), so the two arrows do not cross

# ---- Sensor points scattered on the measured board ----
POINT_COUNT = 9                         # number of drawn sensor points
POINT_ALONG_SPREAD = 0.85               # points lie within +/- this distance of the center along the board
POINT_ACROSS_SPREAD = 0.05              # random scatter perpendicular to the board
POINT_MARKER_SIZE = 4                   # dot size, points
POINT_SEED = 3                          # fixed random seed, so the figure is reproducible

# ---- Normal residual annotation (angle arc) ----
ARC_RADIUS_FRACTION = 0.75               # arc radius as a fraction of the normal arrow length
ARC_RADIUS = ARC_RADIUS_FRACTION * NORMAL_ARROW_LENGTH  # radius of the angle arc, drawing units
ARC_LINE_WIDTH = 1.5                    # arc line width, points
ARC_LABEL_GAP = 0.12                    # horizontal gap between the arc and its label (to the right), drawing units
LABEL_BBOX_PAD = 1.0                    # padding of the surface-colored box that masks rays behind labels, points
REFERENCE_EXTRA = 0.15                  # the dashed reference line extends this much beyond the arc

# ---- Offset residual annotation (dimension line) ----
DIMENSION_LABEL_DROP = 0.42             # the label sits this far below the predicted board's center, under
                                        # the board, joined to the dimension line by a thin leader


def unit(angle_deg):
    """Unit vector at the given angle in degrees, measured counterclockwise from the +x axis."""
    return np.array([np.cos(np.radians(angle_deg)), np.sin(np.radians(angle_deg))])


def board_direction(tilt_deg):
    """Unit vector along a board tilted by tilt_deg from horizontal."""
    return unit(tilt_deg)


def board_normal(tilt_deg):
    """Unit normal of a board, pointing up the page toward the sensor."""
    return unit(tilt_deg + 90.0)


def draw_board(ax, center, tilt_deg, color, label, normal_base_shift):
    """Draw a board as a thick line segment, plus its normal arrow starting normal_base_shift along
    the board from its center; returns the arrow's base point."""
    direction = board_direction(tilt_deg)
    a = center - BOARD_HALF_LENGTH * direction
    b = center + BOARD_HALF_LENGTH * direction
    ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=LINE_WIDTH * BOARD_LINE_FACTOR,
            solid_capstyle="round", label=label, zorder=3)
    base = center + normal_base_shift * direction
    tip = base + NORMAL_ARROW_LENGTH * board_normal(tilt_deg)
    ax.add_patch(FancyArrowPatch(base, tip, arrowstyle="-|>", color=color, lw=LINE_WIDTH,
                                 mutation_scale=ARROW_MUTATION_SCALE, shrinkA=0, shrinkB=0, zorder=4))
    return base


def draw_sensor(ax):
    """Draw the sensor marker at the top and two faint rays for its field of view."""
    ax.plot([SENSOR_POSITION[0]], [SENSOR_POSITION[1]], marker="s", ms=SENSOR_MARKER_SIZE, color=SENSOR_COLOR,
            mec=SURFACE, mew=SENSOR_MARKER_EDGE_WIDTH)
    ax.text(SENSOR_POSITION[0] + SENSOR_LABEL_OFFSET_X, SENSOR_POSITION[1], "sensor", color=INK_SECONDARY,
            va="center", fontsize=LABEL_FONT_SIZE)
    for sign in (-1, 1):
        ax.plot([SENSOR_POSITION[0], sign * RAY_HALF_WIDTH], [SENSOR_POSITION[1], RAY_END_Y], color=GRID,
                lw=THIN_LINE_WIDTH)


def style(ax, title):
    """Match the approach figure: light spines, no ticks, equal aspect, left-aligned caption."""
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, fontsize=TITLE_FONT_SIZE, loc="left")
    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(Y_MIN, Y_MAX)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color(GRID)


fig, ax = plt.subplots(1, 1, figsize=FIGURE_SIZE, dpi=DPI, facecolor=SURFACE)

# Geometry of the two boards. The measured board is rotated by ANGLE_RESIDUAL_DEG and shifted
# along the PREDICTED normal by OFFSET_RESIDUAL. Because the shift is along the predicted
# normal, the measured center is exactly where the dimension line meets the measured plane.
predicted_center = BOARD_CENTER
predicted_tilt = TILT_DEG
predicted_normal = board_normal(predicted_tilt)
measured_tilt = TILT_DEG + ANGLE_RESIDUAL_DEG
measured_center = predicted_center + OFFSET_RESIDUAL * predicted_normal

draw_sensor(ax)

# Sensor points on the measured board, drawn first (under the board line).
rng = np.random.default_rng(POINT_SEED)
along = rng.uniform(-POINT_ALONG_SPREAD, POINT_ALONG_SPREAD, POINT_COUNT)
across = rng.normal(0.0, POINT_ACROSS_SPREAD, POINT_COUNT)
points = (measured_center + np.outer(along, board_direction(measured_tilt))
          + np.outer(across, board_normal(measured_tilt)))
ax.plot(points[:, 0], points[:, 1], linestyle="none", marker="o", ms=POINT_MARKER_SIZE, color=SERIES_APPROACH,
        zorder=2)

draw_board(ax, predicted_center, predicted_tilt, SERIES_TARGET,
           "board as the robot reports it (predicted plane)", PREDICTED_NORMAL_BASE_SHIFT)
measured_base = draw_board(ax, measured_center, measured_tilt, SERIES_APPROACH,
                           "board as the sensor sees it (measured plane)", MEASURED_NORMAL_BASE_SHIFT)

# Normal residual: a dashed reference line parallel to the predicted normal, drawn from the
# measured normal's base, and a small arc between it and the measured normal.
ax.plot([measured_base[0], measured_base[0] + (ARC_RADIUS + REFERENCE_EXTRA) * predicted_normal[0]],
        [measured_base[1], measured_base[1] + (ARC_RADIUS + REFERENCE_EXTRA) * predicted_normal[1]],
        color=SERIES_TARGET, lw=THIN_LINE_WIDTH, linestyle="--", zorder=1)
ax.add_patch(Arc(measured_base, 2 * ARC_RADIUS, 2 * ARC_RADIUS, theta1=predicted_tilt + 90.0,
                 theta2=measured_tilt + 90.0, color=INK, lw=ARC_LINE_WIDTH, zorder=5))
arc_mid_angle = (predicted_tilt + measured_tilt) / 2.0 + 90.0
# The label sits to the right of the arc, level with the arc's midpoint; the measured normal
# arrow leans away to the left, so nothing lies between the arc and the label.
arc_mid = measured_base + ARC_RADIUS * unit(arc_mid_angle)
ax.text(arc_mid[0] + ARC_LABEL_GAP, arc_mid[1], "normal residual (degrees)", color=INK,
        fontsize=LABEL_FONT_SIZE, ha="left", va="center",
        bbox=dict(facecolor=SURFACE, edgecolor="none", pad=LABEL_BBOX_PAD))

# Offset residual: a double-headed dimension line from the predicted board center to the measured
# plane, along the predicted normal, with its label on the left.
ax.add_patch(FancyArrowPatch(predicted_center, measured_center, arrowstyle="<|-|>", color=INK, lw=LINE_WIDTH,
                             mutation_scale=DIMENSION_MUTATION_SCALE, shrinkA=0, shrinkB=0, zorder=5))
dimension_mid = (predicted_center + measured_center) / 2.0
label_anchor = np.array([dimension_mid[0], predicted_center[1] - DIMENSION_LABEL_DROP])
ax.plot([dimension_mid[0], label_anchor[0]], [dimension_mid[1], label_anchor[1]], color=INK,
        lw=THIN_LINE_WIDTH, zorder=4)
ax.text(label_anchor[0], label_anchor[1], "offset residual (mm)", color=INK, fontsize=LABEL_FONT_SIZE,
        ha="center", va="top", bbox=dict(facecolor=SURFACE, edgecolor="none", pad=LABEL_BBOX_PAD))

style(ax, "The two residuals reported for every pose")
ax.legend(loc="lower left", fontsize=LEGEND_FONT_SIZE, frameon=False)

fig.tight_layout()
fig.savefig("fig_residuals.png", facecolor=SURFACE)
print("wrote fig_residuals.png")
