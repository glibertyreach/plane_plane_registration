"""Figure: the board on its adapter, the measurements of section 3, and the board tool frame.

Left (a): side view in section, looking along the board's long edge. The robot flange, the
doweled adapter plate and the board, drawn to scale in millimeters; the distance D from the
flange face to the board's front face; the board tool frame at the center of the front face
(z out of the face toward the sensor, y along the short edge, x toward the viewer); and the
runout check with the dial indicator while joint 6 turns.

Right (b): the board's front face as the sensor sees it: the tool frame axes, the path of the
dial indicator tip during the runout check, the four places where D is measured, and the
three-point mounting (two dowels and a clamp) hidden behind the board.

Same style as the figures of the stage-1 procedure (depth_calibration_from_spherical_target).
Run from this directory:  python3 make_fig_mounting.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle

# ---- Hardware dimensions, millimeters (the drawing is to scale) ----
BOARD_WIDTH_MM = 200.0          # long edge (x of the tool frame)
BOARD_HEIGHT_MM = 150.0         # short edge (y of the tool frame)
BOARD_THICKNESS_MM = 6.0        # the recommended minimum of section 1
ADAPTER_THICKNESS_MM = 20.0     # adapter plate between flange and board
ADAPTER_WIDTH_MM = 80.0         # adapter plate across the section
FLANGE_THICKNESS_MM = 14.0      # drawn depth of the robot flange
FLANGE_WIDTH_MM = 63.0          # a common ISO 9409 flange pattern size
DOWEL_OFFSET_MM = 25.0          # dowel pins sit this far from the flange axis
DOWEL_LENGTH_MM = 16.0          # drawn dowel length across the flange/adapter joint
DOWEL_DIAMETER_MM = 6.0
BOLT_OFFSET_MM = 25.0           # flange bolt distance from the axis (drawn on the other side)
BOLT_LENGTH_MM = 22.0
BOLT_DIAMETER_MM = 6.0
CLAMP_OFFSET_MM = 30.0          # the clamp of the three-point mounting, above the axis
CLAMP_SIZE_MM = (18.0, 10.0)    # clamp foot (width, height) seen from the front

# ---- Section 3 measurements ----
INDICATOR_EDGE_DISTANCE_MM = 20.0     # the dial indicator tip sits this far from the board edge
INDICATOR_TIP_RADIUS_MM = BOARD_HEIGHT_MM / 2 - INDICATOR_EDGE_DISTANCE_MM   # its path radius about the axis
D_MEASUREMENT_INSET_MM = 12.0         # the four D measurements are this far in from the board edges

# ---- Drawing sizes ----
AXIS_ARROW_LENGTH_MM = 45.0           # drawn length of the tool frame axes
VIEWER_AXIS_SYMBOL_RADIUS_MM = 5.0    # the circle of the "toward the viewer" symbol
INDICATOR_BODY_LENGTH_MM = 34.0       # dial indicator stem, from the tip to the dial
INDICATOR_BODY_HEIGHT_MM = 6.0
INDICATOR_DIAL_RADIUS_MM = 14.0
SENSOR_SIZE_MM = (28.0, 44.0)         # the sensor symbol (width, height)
SENSOR_GAP_MM = 240.0                 # from the board face to the sensor symbol (not to scale: the real
                                      # standoff is 550 mm and more)
DIMENSION_OFFSET_MM = 18.0            # dimension lines sit this far outside the part they measure
OUTPUT_DPI = 200
FIGURE_SIZE_IN = (12.0, 3.5)
PANEL_WIDTH_RATIOS = (1.2, 1.0)       # (a) : (b)

# ---- Label placement, millimeters in the drawing's coordinates ----
LABEL_GAP_MM = 8.0                    # gap between a part or an arrow tip and its label
EXTENSION_OVERSHOOT_MM = 4.0          # dimension extension lines overshoot the dimension line by this
AXIS_CENTERLINE_EXTENSION_MM = 60.0   # the flange axis center line extends this far left of the flange
ROTATION_SYMBOL_OFFSET_MM = 36.0      # the joint-6 rotation arrow sits this far left of the flange's outer face
ROTATION_SYMBOL_HALF_HEIGHT_MM = 14.0
LABEL_DOWEL_XY = (-95.0, 68.0)        # text position of the dowel label (side view)
LABEL_BOLT_XY = (-70.0, -52.0)        # text position of the bolt label (side view)
LABEL_FLANGE_XY = (-60.0, -75.0)      # text position of the robot flange label (side view)
LABEL_ADAPTER_XY = (-48.0, -100.0)     # text position of the adapter label (side view)
VIEWER_SYMBOL_OFFSET_MM = (16.0, -24.0)   # the x-axis symbol, relative to the front face center (side view)
LABEL_FRAME_XY = (40.0, 82.0)         # the tool frame note, relative to the front face center (side view)
VIEWING_ARROW_GAP_MM = 22.0           # the viewing-direction arrow stops this far beyond the z arrow tip
SIDE_VIEW_XLIM = (-215.0, 50.0)       # left limit, and right margin beyond the sensor symbol
SIDE_VIEW_YLIM = (-125.0, 130.0)
FRONT_DOWEL_Y_MM = 18.0               # the two hidden dowels sit this far above the center (front view)
FRONT_Z_SYMBOL_XY = (-30.0, -30.0)    # the z-axis symbol (front view)
INDICATOR_LEADER_ANGLE_DEG = 70.0     # where the indicator-path leader touches the dotted circle
LABEL_INDICATOR_PATH_RISE_MM = 22.0   # the indicator-path label sits this far above the board's top edge
LABEL_MOUNTING_Y_MM = -60.0           # height of the three-point mounting note, right of the board
LABEL_D_PLACES_Y_MM = -5.0            # height of the D-places note, right of the board
FRONT_VIEW_MARGINS_MM = (95.0, 245.0, 55.0, 60.0)   # left, right, bottom, top margins around the board
FONT_SIZE = 8
TITLE_FONT_SIZE = 10

# ---- Colors (as the stage-1 figures) ----
STEEL = "#8c8c8c"
DARK = "#444444"
BOARD_COLOR = "#dcdcdc"
ACCENT = "#b03030"
BLUE = "#1f5fa8"
HATCH = "////"


def dimension(ax, p0, p1, text, text_offset, ha="center", va="bottom"):
    """A double-headed dimension line from p0 to p1 with its text offset from the midpoint."""
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="<|-|>", color=DARK, lw=0.9, mutation_scale=8,
                                 shrinkA=0, shrinkB=0))
    mid = (np.asarray(p0) + np.asarray(p1)) / 2 + np.asarray(text_offset)
    ax.text(mid[0], mid[1], text, ha=ha, va=va, fontsize=FONT_SIZE, color=DARK)


def axis_arrow(ax, origin, direction, label, label_offset):
    """A tool frame axis drawn in the accent color."""
    tip = np.asarray(origin) + AXIS_ARROW_LENGTH_MM * np.asarray(direction)
    ax.add_patch(FancyArrowPatch(origin, tip, arrowstyle="-|>", color=ACCENT, lw=1.6, mutation_scale=12,
                                 shrinkA=0, shrinkB=0, zorder=6))
    ax.text(tip[0] + label_offset[0], tip[1] + label_offset[1], label, color=ACCENT, fontsize=FONT_SIZE + 1,
            ha="center", va="center", zorder=6)


def viewer_axis_symbol(ax, center, toward_viewer, label, label_offset):
    """The symbol of an axis perpendicular to the page: a dot in a circle (toward the viewer) or a cross."""
    ax.add_patch(Circle(center, VIEWER_AXIS_SYMBOL_RADIUS_MM, facecolor="white", edgecolor=ACCENT, lw=1.4, zorder=6))
    if toward_viewer:
        ax.plot([center[0]], [center[1]], marker="o", ms=3, color=ACCENT, zorder=7)
    else:
        r = VIEWER_AXIS_SYMBOL_RADIUS_MM / np.sqrt(2)
        ax.plot([center[0] - r, center[0] + r], [center[1] - r, center[1] + r], color=ACCENT, lw=1.2, zorder=7)
        ax.plot([center[0] - r, center[0] + r], [center[1] + r, center[1] - r], color=ACCENT, lw=1.2, zorder=7)
    ax.text(center[0] + label_offset[0], center[1] + label_offset[1], label, color=ACCENT, fontsize=FONT_SIZE,
            ha="left", va="center", zorder=6)


def draw_side_view(ax):
    """(a) flange, adapter and board in section, D, the tool frame and the runout check."""
    flange_face_x = 0.0
    flange_outer_x = flange_face_x - FLANGE_THICKNESS_MM
    adapter_x0 = flange_face_x
    board_x0 = adapter_x0 + ADAPTER_THICKNESS_MM
    face_x = board_x0 + BOARD_THICKNESS_MM          # the front face: D from the flange face
    # Parts.
    ax.add_patch(Rectangle((flange_outer_x, -FLANGE_WIDTH_MM / 2), FLANGE_THICKNESS_MM, FLANGE_WIDTH_MM, color=DARK))
    ax.add_patch(Rectangle((adapter_x0, -ADAPTER_WIDTH_MM / 2), ADAPTER_THICKNESS_MM, ADAPTER_WIDTH_MM,
                           facecolor="white", edgecolor=DARK, hatch=HATCH, lw=1.0))
    ax.add_patch(Rectangle((board_x0, -BOARD_HEIGHT_MM / 2), BOARD_THICKNESS_MM, BOARD_HEIGHT_MM,
                           facecolor=BOARD_COLOR, edgecolor=DARK, lw=1.0))
    ax.plot([face_x, face_x], [-BOARD_HEIGHT_MM / 2, BOARD_HEIGHT_MM / 2], color=ACCENT, lw=2.0, zorder=5)
    # The flange axis as a center line, extended to the left for the rotation symbol.
    ax.plot([flange_outer_x - AXIS_CENTERLINE_EXTENSION_MM, board_x0], [0.0, 0.0], color=DARK, lw=0.6,
            linestyle=(0, (6, 2, 1, 2)), zorder=1)
    # Dowel pin (above the axis) and a flange bolt (below), crossing the flange/adapter joint.
    ax.add_patch(Rectangle((flange_face_x - DOWEL_LENGTH_MM / 2, DOWEL_OFFSET_MM - DOWEL_DIAMETER_MM / 2),
                           DOWEL_LENGTH_MM, DOWEL_DIAMETER_MM, facecolor="white", edgecolor=DARK, lw=0.8, zorder=4))
    ax.add_patch(Rectangle((flange_outer_x, -BOLT_OFFSET_MM - BOLT_DIAMETER_MM / 2),
                           BOLT_LENGTH_MM, BOLT_DIAMETER_MM, facecolor="white", edgecolor=DARK, lw=0.8, zorder=4))
    # Labels of the parts. Leaders start on the flange's outer face so they do not cross the flange.
    ax.annotate("dowel pin (locates the adapter)", (flange_outer_x, DOWEL_OFFSET_MM), LABEL_DOWEL_XY,
                fontsize=FONT_SIZE, color=DARK, ha="center", va="bottom",
                arrowprops=dict(arrowstyle="-", color=DARK, lw=0.7))
    ax.annotate("flange bolt (one of four)", (flange_outer_x, -BOLT_OFFSET_MM), LABEL_BOLT_XY,
                fontsize=FONT_SIZE, color=DARK, ha="right", va="center",
                arrowprops=dict(arrowstyle="-", color=DARK, lw=0.7))
    ax.annotate("robot flange", (flange_outer_x + FLANGE_THICKNESS_MM / 2, -FLANGE_WIDTH_MM / 2), LABEL_FLANGE_XY,
                fontsize=FONT_SIZE, color=DARK, ha="right", va="top",
                arrowprops=dict(arrowstyle="-", color=DARK, lw=0.7))
    ax.annotate("adapter plate", (adapter_x0 + ADAPTER_THICKNESS_MM / 2, -ADAPTER_WIDTH_MM / 2), LABEL_ADAPTER_XY,
                fontsize=FONT_SIZE, color=DARK, ha="center", va="top",
                arrowprops=dict(arrowstyle="-", color=DARK, lw=0.7))
    ax.text(board_x0 + BOARD_THICKNESS_MM / 2, -BOARD_HEIGHT_MM / 2 - LABEL_GAP_MM, "board\n(6 mm)", ha="center",
            va="top", fontsize=FONT_SIZE, color=DARK)
    # Joint 6 rotation: a curved arrow around the extended flange axis.
    arc_x = flange_outer_x - ROTATION_SYMBOL_OFFSET_MM
    ax.add_patch(FancyArrowPatch((arc_x, -ROTATION_SYMBOL_HALF_HEIGHT_MM), (arc_x, ROTATION_SYMBOL_HALF_HEIGHT_MM),
                                 connectionstyle="arc3,rad=0.8", arrowstyle="-|>", color=DARK, lw=1.0,
                                 mutation_scale=10))
    ax.text(arc_x - LABEL_GAP_MM, ROTATION_SYMBOL_HALF_HEIGHT_MM + LABEL_GAP_MM,
            "turn joint 6 through 360 degrees:\nthe reading must stay within 0.05 mm", ha="right", va="bottom",
            fontsize=FONT_SIZE, color=DARK)
    # D: flange face to front face, dimensioned above the parts with extension lines.
    dim_y = BOARD_HEIGHT_MM / 2 + DIMENSION_OFFSET_MM
    ax.plot([flange_face_x, flange_face_x], [ADAPTER_WIDTH_MM / 2, dim_y + EXTENSION_OVERSHOOT_MM], color=DARK, lw=0.6)
    ax.plot([face_x, face_x], [BOARD_HEIGHT_MM / 2, dim_y + EXTENSION_OVERSHOOT_MM], color=DARK, lw=0.6)
    dimension(ax, (flange_face_x, dim_y), (face_x, dim_y), "D: flange face to front face\n(measure at four places, b)",
              (0.0, LABEL_GAP_MM))
    # Tool frame at the center of the front face.
    origin = (face_x, 0.0)
    ax.plot([origin[0]], [origin[1]], marker="+", color=ACCENT, markersize=12, mew=2, zorder=7)
    axis_arrow(ax, origin, (1.0, 0.0), "z", (LABEL_GAP_MM, 0.0))
    axis_arrow(ax, origin, (0.0, 1.0), "y", (LABEL_GAP_MM / 2, LABEL_GAP_MM))
    viewer_axis_symbol(ax, (face_x + VIEWER_SYMBOL_OFFSET_MM[0], VIEWER_SYMBOL_OFFSET_MM[1]), True,
                       "x: toward you, along the long edge", (VIEWER_AXIS_SYMBOL_RADIUS_MM + 4.0, 0.0))
    ax.text(face_x + LABEL_FRAME_XY[0], LABEL_FRAME_XY[1], "tool frame origin: center of the front face;\n"
            "z = outward normal, toward the sensor", color=ACCENT, fontsize=FONT_SIZE, ha="left", va="center")
    # Runout check: dial indicator tip on the front face, 20 mm from the bottom edge.
    tip_y = -(BOARD_HEIGHT_MM / 2 - INDICATOR_EDGE_DISTANCE_MM)
    ax.add_patch(Rectangle((face_x + 1.0, tip_y - INDICATOR_BODY_HEIGHT_MM / 2), INDICATOR_BODY_LENGTH_MM,
                           INDICATOR_BODY_HEIGHT_MM, color=BLUE, zorder=5))
    dial_x = face_x + 1.0 + INDICATOR_BODY_LENGTH_MM + INDICATOR_DIAL_RADIUS_MM
    ax.add_patch(Circle((dial_x, tip_y), INDICATOR_DIAL_RADIUS_MM, facecolor="white", edgecolor=BLUE, lw=1.8,
                        zorder=5))
    ax.text(dial_x + INDICATOR_DIAL_RADIUS_MM + LABEL_GAP_MM, tip_y,
            "dial indicator on a magnetic base,\ntip on the front face 20 mm from the edge", color=BLUE,
            fontsize=FONT_SIZE, ha="left", va="center")
    # Sensor, to the right (its distance is not to scale).
    sensor_x = face_x + SENSOR_GAP_MM
    ax.add_patch(Rectangle((sensor_x, -SENSOR_SIZE_MM[1] / 2), SENSOR_SIZE_MM[0], SENSOR_SIZE_MM[1], color=BLUE))
    ax.text(sensor_x + SENSOR_SIZE_MM[0] / 2, SENSOR_SIZE_MM[1] / 2 + LABEL_GAP_MM,
            "sensor\n(its distance is not to scale)", ha="center", va="bottom", fontsize=FONT_SIZE, color=BLUE)
    ax.add_patch(FancyArrowPatch((sensor_x - LABEL_GAP_MM, 0.0), (face_x + AXIS_ARROW_LENGTH_MM + VIEWING_ARROW_GAP_MM,
                                 0.0), arrowstyle="-|>", color=BLUE, lw=1.0, mutation_scale=10))
    ax.text(sensor_x - LABEL_GAP_MM, LABEL_GAP_MM / 2, "viewing direction", ha="right", va="bottom",
            fontsize=FONT_SIZE, color=BLUE)
    ax.set_xlim(SIDE_VIEW_XLIM[0], sensor_x + SIDE_VIEW_XLIM[1])
    ax.set_ylim(*SIDE_VIEW_YLIM)
    ax.set_title("(a) flange, adapter and board in section: D, the tool frame, the runout check",
                 fontsize=TITLE_FONT_SIZE, loc="left")


def draw_front_view(ax):
    """(b) the front face as the sensor sees it: axes, indicator path, D places, hidden mounting."""
    half_w, half_h = BOARD_WIDTH_MM / 2, BOARD_HEIGHT_MM / 2
    ax.add_patch(Rectangle((-half_w, -half_h), BOARD_WIDTH_MM, BOARD_HEIGHT_MM, facecolor=BOARD_COLOR,
                           edgecolor=ACCENT, lw=1.6))
    # Hidden mounting behind the board: the adapter outline, two dowels and the clamp (dashed).
    ax.add_patch(Rectangle((-ADAPTER_WIDTH_MM / 2, -ADAPTER_WIDTH_MM / 2), ADAPTER_WIDTH_MM, ADAPTER_WIDTH_MM,
                           facecolor="none", edgecolor=DARK, lw=0.8, linestyle="--"))
    for sign in (-1, 1):
        ax.add_patch(Circle((sign * DOWEL_OFFSET_MM, FRONT_DOWEL_Y_MM), DOWEL_DIAMETER_MM / 2,
                            facecolor="none", edgecolor=DARK, lw=0.8, linestyle="--"))
    ax.add_patch(Rectangle((-CLAMP_SIZE_MM[0] / 2, -CLAMP_OFFSET_MM - CLAMP_SIZE_MM[1] / 2), CLAMP_SIZE_MM[0],
                           CLAMP_SIZE_MM[1], facecolor="none", edgecolor=DARK, lw=0.8, linestyle="--"))
    ax.annotate("behind the board (dashed): adapter plate,\ntwo dowels and one clamp = three-point mounting",
                (ADAPTER_WIDTH_MM / 2, -ADAPTER_WIDTH_MM / 2), (half_w + LABEL_GAP_MM, LABEL_MOUNTING_Y_MM),
                fontsize=FONT_SIZE, color=DARK, ha="left", va="center",
                arrowprops=dict(arrowstyle="-", color=DARK, lw=0.7))
    # Tool frame: x along the long edge, y along the short edge, z toward the sensor (toward the viewer).
    origin = (0.0, 0.0)
    ax.plot([0.0], [0.0], marker="+", color=ACCENT, markersize=12, mew=2, zorder=7)
    axis_arrow(ax, origin, (1.0, 0.0), "x", (LABEL_GAP_MM, 0.0))
    axis_arrow(ax, origin, (0.0, 1.0), "y", (0.0, LABEL_GAP_MM))
    ax.add_patch(Circle(FRONT_Z_SYMBOL_XY, VIEWER_AXIS_SYMBOL_RADIUS_MM, facecolor="white", edgecolor=ACCENT, lw=1.4,
                        zorder=6))
    ax.plot([FRONT_Z_SYMBOL_XY[0]], [FRONT_Z_SYMBOL_XY[1]], marker="o", ms=3, color=ACCENT, zorder=7)
    ax.text(FRONT_Z_SYMBOL_XY[0] - VIEWER_AXIS_SYMBOL_RADIUS_MM - 4.0, FRONT_Z_SYMBOL_XY[1], "z: toward the sensor",
            color=ACCENT, fontsize=FONT_SIZE, ha="right", va="center", zorder=6,
            bbox=dict(facecolor=BOARD_COLOR, edgecolor="none", pad=1.0))
    # Path of the dial indicator tip while joint 6 turns.
    ax.add_patch(Circle((0.0, 0.0), INDICATOR_TIP_RADIUS_MM, facecolor="none", edgecolor=BLUE, lw=1.2,
                        linestyle=":"))
    ax.plot([0.0], [-INDICATOR_TIP_RADIUS_MM], marker="o", ms=6, color=BLUE, zorder=6)
    path_point = INDICATOR_TIP_RADIUS_MM * np.array([np.cos(np.radians(INDICATOR_LEADER_ANGLE_DEG)),
                                                     np.sin(np.radians(INDICATOR_LEADER_ANGLE_DEG))])
    ax.annotate("path of the dial indicator tip as joint 6 turns\n(the tip stays 20 mm from the edge)",
                path_point, (0.0, half_h + LABEL_INDICATOR_PATH_RISE_MM), fontsize=FONT_SIZE, color=BLUE,
                ha="center", va="bottom", arrowprops=dict(arrowstyle="-", color=BLUE, lw=0.7))
    # The four places where D is measured: near the four corners.
    places = [(sx * (half_w - D_MEASUREMENT_INSET_MM), sy * (half_h - D_MEASUREMENT_INSET_MM))
              for sx in (-1, 1) for sy in (-1, 1)]
    for x, y in places:
        ax.plot([x], [y], marker="s", ms=6, color=DARK, mec="white", mew=0.8, zorder=6)
    ax.annotate("D measured here (four places;\nthey must agree within 0.05 mm)", places[3],
                (half_w + LABEL_GAP_MM, LABEL_D_PLACES_Y_MM), fontsize=FONT_SIZE, color=DARK, ha="left",
                va="center", arrowprops=dict(arrowstyle="-", color=DARK, lw=0.7))
    # Outer dimensions.
    dimension(ax, (-half_w, -half_h - DIMENSION_OFFSET_MM), (half_w, -half_h - DIMENSION_OFFSET_MM),
              "200 mm (long edge = x)", (0.0, -LABEL_GAP_MM / 2), va="top")
    dimension(ax, (-half_w - DIMENSION_OFFSET_MM, -half_h), (-half_w - DIMENSION_OFFSET_MM, half_h),
              "150 mm\n(short edge = y)", (-LABEL_GAP_MM, 0.0), ha="right", va="center")
    ax.set_xlim(-half_w - FRONT_VIEW_MARGINS_MM[0], half_w + FRONT_VIEW_MARGINS_MM[1])
    ax.set_ylim(-half_h - FRONT_VIEW_MARGINS_MM[2], half_h + FRONT_VIEW_MARGINS_MM[3])
    ax.set_title("(b) the front face as the sensor sees it", fontsize=TITLE_FONT_SIZE, loc="left")


def main() -> None:
    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=FIGURE_SIZE_IN, dpi=OUTPUT_DPI,
                                     gridspec_kw={"width_ratios": list(PANEL_WIDTH_RATIOS)})
    draw_side_view(ax_a)
    draw_front_view(ax_b)
    for ax in (ax_a, ax_b):
        ax.set_aspect("equal")
        ax.axis("off")
    fig.tight_layout()
    fig.savefig("fig_mounting.png", dpi=OUTPUT_DPI, facecolor="white")
    print("wrote fig_mounting.png")


if __name__ == "__main__":
    main()
