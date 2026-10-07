"""Figure: the two sub-procedures of the registration captures.

Left: sub-procedure A, "ignoring backlash": one move straight onto each target pose.
Right: sub-procedure B, "minimizing backlash": a move to the approach pose (the target
retreated along the board normal and rotated slightly about the board's x axis), then the
same final move onto every target. Drawn in the sensor's view plane for one target pose.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
GRID = "#e6e5e1"
SERIES_TARGET = "#2a78d6"     # target pose and final move
SERIES_APPROACH = "#eb6834"   # approach pose and the move to it
SENSOR_COLOR = "#52514e"
LINE_WIDTH = 2.0
DPI = 150
BOARD_HALF_LENGTH = 1.0       # drawing units
RETREAT = 0.9                 # the approach retreat, drawing units
ROTATION_DEG = 12.0           # exaggerated approach rotation for legibility (the plan default is 3 deg)
TILT_DEG = 20.0               # tilt of the drawn target board
NORMAL_ARROW_LENGTH = 0.35    # length of the drawn board normals, drawing units


def board_segment(center, angle_deg):
    direction = np.array([np.cos(np.radians(angle_deg)), np.sin(np.radians(angle_deg))])
    return center - BOARD_HALF_LENGTH * direction, center + BOARD_HALF_LENGTH * direction


def draw_board(ax, center, angle_deg, color, label):
    a, b = board_segment(center, angle_deg)
    ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=LINE_WIDTH * 2, solid_capstyle="round", label=label)
    normal = np.array([-np.sin(np.radians(angle_deg)), np.cos(np.radians(angle_deg))])
    ax.add_patch(FancyArrowPatch(center, center + NORMAL_ARROW_LENGTH * normal, arrowstyle="-|>", color=color,
                                 lw=LINE_WIDTH, mutation_scale=12))


def draw_sensor(ax):
    ax.plot([0.0], [3.6], marker="s", ms=10, color=SENSOR_COLOR, mec=SURFACE, mew=2)
    ax.text(0.15, 3.6, "sensor", color=INK_SECONDARY, va="center", fontsize=9)
    for sign in (-1, 1):
        ax.plot([0.0, sign * 2.2], [3.6, 0.0], color=GRID, lw=1)


def style(ax, title):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, fontsize=10, loc="left")
    ax.set_xlim(-2.6, 2.6)
    ax.set_ylim(-0.6, 4.0)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color(GRID)


fig, axes = plt.subplots(1, 2, figsize=(9, 4), dpi=DPI, facecolor=SURFACE)
target_center = np.array([0.4, 1.2])
# The board faces the sensor: its normal points up the page toward the sensor marker, tilted.
board_angle = TILT_DEG
normal = np.array([-np.sin(np.radians(board_angle)), np.cos(np.radians(board_angle))])

# Panel A: a single move from the previous pose straight onto the target.
ax = axes[0]
draw_sensor(ax)
draw_board(ax, target_center, board_angle, SERIES_TARGET, "target pose")
previous = np.array([-1.6, 0.4])
ax.plot([previous[0]], [previous[1]], marker="o", ms=8, color=INK_SECONDARY, mec=SURFACE, mew=2)
ax.text(previous[0] - 0.1, previous[1] - 0.3, "previous pose", color=INK_SECONDARY, fontsize=8, ha="center")
ax.add_patch(FancyArrowPatch(previous, target_center, arrowstyle="-|>", color=SERIES_TARGET, lw=LINE_WIDTH,
                             mutation_scale=14, linestyle="--"))
ax.text(-0.9, 1.1, "one move,\nany direction", color=INK_SECONDARY, fontsize=8, ha="center")
style(ax, "A  ignoring backlash")
ax.legend(loc="lower right", fontsize=8, frameon=False)

# Panel B: previous pose -> approach pose -> the same final move onto the target.
ax = axes[1]
draw_sensor(ax)
approach_center = target_center - RETREAT * normal
draw_board(ax, approach_center, board_angle - ROTATION_DEG, SERIES_APPROACH, "approach pose")
draw_board(ax, target_center, board_angle, SERIES_TARGET, "target pose")
ax.plot([previous[0]], [previous[1]], marker="o", ms=8, color=INK_SECONDARY, mec=SURFACE, mew=2)
ax.text(previous[0] - 0.1, previous[1] - 0.3, "previous pose", color=INK_SECONDARY, fontsize=8, ha="center")
ax.add_patch(FancyArrowPatch(previous, approach_center, arrowstyle="-|>", color=SERIES_APPROACH, lw=LINE_WIDTH,
                             mutation_scale=14, linestyle="--"))
ax.add_patch(FancyArrowPatch(approach_center, target_center, arrowstyle="-|>", color=SERIES_TARGET,
                             lw=LINE_WIDTH, mutation_scale=14))
ax.text(-2.45, 2.9, "the same final move at every pose:\nthe retreat distance back along the\nboard normal, plus a small rotation",
        color=INK_SECONDARY, fontsize=8, ha="left", va="top")
style(ax, "B  minimizing backlash")
ax.legend(loc="lower right", fontsize=8, frameon=False)

fig.tight_layout()
fig.savefig("fig_approach.png", facecolor=SURFACE)
print("wrote fig_approach.png")
