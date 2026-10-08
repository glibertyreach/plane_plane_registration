"""
Residual figures of a registration (code design 8.2).

    python3 -m planereg.analysis.residual_maps --registration registration.json --out DIR
        [--model rigid|similarity|both] [--pixel-maps N] [--pixel-maps-random M]

``register`` calls :func:`make_figures` itself; this module also runs on its own from a registration.json and
the segmentation.npz next to it (and, for the per-pixel maps, the capture files of the manifest it names).

Figures, per model (the result of the final pass), PNG at 150 dpi
    pose_residuals.png          normal residual (deg) and offset residual (mm) against pose index, two
                                panels; poses that took part in the solve and rejected or held-out poses are distinguished
    normal_residual_field.png   vector map over the image: an arrow at every board center for the normal
                                residual vector R n_k - m_k seen in the sensor's image axes, one panel per
                                standoff; a quiver key states the arrow length in degrees
    offset_residual_field.png   a marker at every board center colored by the signed offset residual, one panel
                                per standoff
    pixel_residuals_<pose_id>.png
                                for the poses with the largest |offset residual| and a few random ones: the
                                signed distance of every mask pixel to the plane predicted from the robot pose
                                through the final transform, a quiver of the in-image gradient of that residual
                                (the local tilt error, key in degrees) and the mask outline
and, from ``compare``, ``comparison.png`` (:func:`plot_comparison`).

Style (the rules of the design). Light surface; text in ink colors, never in a series color; hairline
gridlines; a legend whenever a panel holds two or more series; 2 px lines; markers of at least 8 px with a 2 px
surface-colored edge; a single-hue sequential map ('Blues') for magnitudes and standoffs; a diverging map
through a gray midpoint, symmetric about zero, for signed residuals; separate panels instead of a second axis;
every axis labeled with its unit. matplotlib is imported lazily; without it the functions print a WARNING and
write nothing.

Frames and units: image pixels (u, v) = (column, row), v downward; the sensor-frame normal residual vector has
x along u and y along v; degrees, millimeters.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np
from scipy import ndimage

from planereg.analysis.register import MASK_KEY_FORMAT, PASS_ONE, PASS_TWO, load_registration, transform_from_block
from planereg.core.planes import board_plane_in_base, predicted_sensor_plane
from sphcal.features.depth_features import temporal_mean_points
from sphcal.io.capture_set import CaptureSet
from sphcal.io.poses import load_manifest

# ---------------------------------------------------------------------------
# Constants: exit codes, files
# ---------------------------------------------------------------------------
EXIT_OK = 0
EXIT_INPUT_ERROR = 2
"""Exit codes: figures written (or matplotlib absent, with a WARNING); an input must be fixed."""

POSE_RESIDUALS_FILE = "pose_residuals.png"
NORMAL_FIELD_FILE = "normal_residual_field.png"
OFFSET_FIELD_FILE = "offset_residual_field.png"
PIXEL_RESIDUALS_FILE_FORMAT = "pixel_residuals_{pose_id}.png"
COMPARISON_FILE = "comparison.png"
"""File names of the figures."""

# ---------------------------------------------------------------------------
# Constants: figure style (design 8.2)
# ---------------------------------------------------------------------------
FIGURE_DPI = 150
"""Resolution of every PNG."""
POINTS_PER_INCH = 72.0
"""Typographic points per inch, to express pixel sizes in matplotlib's point units."""
SURFACE_COLOR = "#fcfcfb"
INK_PRIMARY = "#0b0b0b"
INK_SECONDARY = "#52514e"
GRIDLINE_COLOR = "#e6e5e1"
"""Light surface, text colors (primary, secondary) and hairline gridlines."""
SERIES_COLOR_BLUE = "#2a78d6"
SERIES_COLOR_ORANGE = "#eb6834"
"""First two categorical series colors: poses used / procedure A, rejected poses / procedure B."""
DIVERGING_NEGATIVE_COLOR = "#256abf"
DIVERGING_MIDPOINT_COLOR = "#d3d2ce"
DIVERGING_POSITIVE_COLOR = "#c83a2c"
"""Poles and gray midpoint of the diverging map of signed residuals (negative blue, positive red)."""
DIVERGING_MAP_NAME = "residual_diverging"
SEQUENTIAL_MAP_NAME = "Blues"
SEQUENTIAL_RANGE = (0.45, 0.95)
"""The part of the Blues map used for discrete standoffs, light (near) to dark (far); the lightest end stays
clear of the surface."""
LINE_WIDTH_PX = 2.0
HAIRLINE_PX = 1.0
MARKER_DIAMETER_PX = 10.0
MARKER_EDGE_PX = 2.0
"""Line width, gridline width, marker diameter (at least 8 px after the edge), and the surface-colored marker
edge, all in pixels of the saved PNG."""
FONT_SIZE_PT = 9.0
TITLE_PAD_PT = 26.0
"""Base font size, and the room left above an axes for a quiver key under its title."""
LEGEND_LOCATION = "outside lower center"
QUIVER_KEY_POSITION = (0.2, 1.03)
"""Figure legend location (below the panels); the quiver key's anchor in axes coordinates."""
THRESHOLD_LINE_STYLE = (0, (4, 3))
"""Dash pattern of acceptance-limit lines."""
MARKER_CIRCLE = "o"
MARKER_DIAMOND = "D"
"""Marker shapes: poses that took part, rejected poses."""

# Guards and tolerances
ARROW_LENGTH_FLOOR_DEG = 1e-9
"""Arrow lengths below this are treated as zero when scaling the quiver."""
SYMMETRIC_LIMIT_FLOOR = 1e-6
"""Floor on the half-range of a symmetric color scale, so an all-zero map still has a valid scale."""
SMOOTHING_WEIGHT_FLOOR = 1e-6
"""Smoothed mask weights below this count as 'no mask nearby'."""
NICE_MANTISSAS = (1.0, 2.0, 5.0)
"""Mantissas of the round numbers a quiver key may take."""
DECIMAL_BASE = 10.0
"""Base of the decades in which round numbers are chosen."""
MASK_CONTOUR_LEVEL = 0.5
"""Contour level of a boolean mask converted to float."""
HISTOGRAM_BIN_FLOOR = 1
"""Fewest histogram bins."""
HISTOGRAM_FILL_ALPHA = 0.25
"""Opacity of the filled part of an overlaid histogram; the outline is opaque."""
PIXEL_HALF_EXTENT = 0.5
"""Half the width of a pixel: image extents run from the first pixel's edge to the last pixel's edge."""


@dataclass(frozen=True)
class FigureParameters:
    """Every number of the figures that is not a style rule."""

    pixel_map_count: int = 6
    """Per-pixel maps of this many poses with the largest |offset residual| ..."""
    pixel_map_random_count: int = 2
    """... plus this many chosen at random from the others."""
    seed: int = 0
    """Seed of the random choice."""
    standoff_gap_mm: float = 100.0
    """Board centers whose depths differ by less than this belong to one standoff."""
    max_standoff_panels: int = 6
    """More standoff groups than this are drawn in one panel (the standoffs are then not discrete)."""
    panel_width_in: float = 5.0
    panel_height_in: float = 4.4
    """Size of one panel."""
    panels_per_row: int = 3
    """Most panels side by side."""
    arrow_fraction_of_width: float = 0.15
    """The longest arrow of a vector map is this fraction of the image width."""
    quiver_step_px: int = 8
    """Spacing of the arrows of a per-pixel map."""
    gradient_smoothing_px: float = 3.0
    """Gaussian sigma of the smoothing applied to a per-pixel residual before its gradient is taken."""
    crop_margin_px: int = 8
    """Margin around the mask in a per-pixel map."""
    histogram_bins: int = 20


# ---------------------------------------------------------------------------
# matplotlib access and style
# ---------------------------------------------------------------------------
def load_pyplot():
    """matplotlib.pyplot with a non-interactive backend, or None after printing a WARNING when matplotlib is
    not installed."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARNING: matplotlib is not installed, so no figure was written; install it with "
              "'pip install matplotlib' (or planereg[figures])")
        return None
    return plt


def pixels_to_points(pixels: float) -> float:
    """A length in pixels of the saved PNG as matplotlib points."""
    return pixels * POINTS_PER_INCH / FIGURE_DPI


def style_rc() -> dict:
    """rcParams that realize the design's style rules."""
    return {
        "figure.dpi": FIGURE_DPI, "savefig.dpi": FIGURE_DPI,
        "figure.facecolor": SURFACE_COLOR, "axes.facecolor": SURFACE_COLOR, "savefig.facecolor": SURFACE_COLOR,
        "text.color": INK_PRIMARY, "axes.labelcolor": INK_PRIMARY, "axes.titlecolor": INK_PRIMARY,
        "xtick.color": INK_SECONDARY, "ytick.color": INK_SECONDARY,
        "xtick.labelcolor": INK_SECONDARY, "ytick.labelcolor": INK_SECONDARY,
        "axes.edgecolor": GRIDLINE_COLOR, "axes.grid": True, "axes.axisbelow": True,
        "grid.color": GRIDLINE_COLOR, "grid.linewidth": pixels_to_points(HAIRLINE_PX),
        "lines.linewidth": pixels_to_points(LINE_WIDTH_PX),
        "legend.frameon": False, "legend.labelcolor": INK_PRIMARY,
        "font.size": FONT_SIZE_PT,
    }


def diverging_colormap():
    """Blue - gray - red through a visible gray midpoint (zero residual)."""
    from matplotlib.colors import LinearSegmentedColormap
    return LinearSegmentedColormap.from_list(
        DIVERGING_MAP_NAME, [DIVERGING_NEGATIVE_COLOR, DIVERGING_MIDPOINT_COLOR, DIVERGING_POSITIVE_COLOR])


def standoff_colors(plt, count: int) -> list:
    """``count`` colors from the single-hue Blues map, light (near) to dark (far)."""
    colormap = plt.get_cmap(SEQUENTIAL_MAP_NAME)
    positions = [float(np.mean(SEQUENTIAL_RANGE))] if count == 1 else np.linspace(*SEQUENTIAL_RANGE, count)
    return [colormap(position) for position in positions]


def marker_area_pt2() -> float:
    """Scatter marker area (points squared) of a MARKER_DIAMETER_PX circle."""
    return pixels_to_points(MARKER_DIAMETER_PX) ** 2


def new_figure(plt, rows: int, columns: int, params: FigureParameters):
    """A constrained-layout figure of rows x columns panels (always a 2-D array of axes)."""
    return plt.subplots(rows, columns, figsize=(params.panel_width_in * columns, params.panel_height_in * rows),
                        squeeze=False, layout="constrained")


def save_figure(plt, figure, path: Path) -> Path:
    """Write the figure as a PNG at the design's resolution and close it."""
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=FIGURE_DPI)
    plt.close(figure)
    return path


def add_legend(figure, handles: list) -> None:
    """One legend for the whole figure, below the panels where it covers no data, when there are two or more
    series (the design rule); nothing for one."""
    if len(handles) >= 2:
        figure.legend(handles=handles, loc=LEGEND_LOCATION, ncol=len(handles))


def marker_handle(label: str, color: str, marker: str, hollow: bool = False):
    """A legend handle that looks like a plotted marker (surface-colored edge, or a hollow ring)."""
    from matplotlib.lines import Line2D
    return Line2D([], [], linestyle="none", marker=marker, markersize=pixels_to_points(MARKER_DIAMETER_PX),
                  markerfacecolor=SURFACE_COLOR if hollow else color,
                  markeredgecolor=color if hollow else SURFACE_COLOR,
                  markeredgewidth=pixels_to_points(MARKER_EDGE_PX), label=label)


def line_handle(label: str, color: str, style="-"):
    """A legend handle that looks like a plotted 2 px line."""
    from matplotlib.lines import Line2D
    return Line2D([], [], color=color, linestyle=style, linewidth=pixels_to_points(LINE_WIDTH_PX), label=label)


# ---------------------------------------------------------------------------
# Data helpers
# ---------------------------------------------------------------------------
def segmented_poses(block: dict) -> list[dict]:
    """The pose entries of a result block that were segmented and carry a residual."""
    return [pose for pose in block["poses"]
            if pose["segmented"] and pose["normal_residual_deg"] is not None]


def group_standoffs(depths_mm: np.ndarray, gap_mm: float) -> np.ndarray:
    """Group index (0 = nearest) of every depth: sorted depths start a new group wherever consecutive values
    differ by more than ``gap_mm``."""
    order = np.argsort(depths_mm)
    labels_sorted = np.concatenate([[0], np.cumsum(np.diff(depths_mm[order]) > gap_mm)]).astype(int)
    labels = np.empty_like(labels_sorted)
    labels[order] = labels_sorted
    return labels


def nice_length(value: float) -> float:
    """The largest of 1, 2, 5 times a power of ten that does not exceed ``value`` (> 0)."""
    decade = DECIMAL_BASE ** np.floor(np.log10(value))
    return float(max(mantissa * decade for mantissa in NICE_MANTISSAS if mantissa * decade <= value))


def symmetric_limit(values: np.ndarray) -> float:
    """Half-range of a color scale symmetric about zero that covers the values."""
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    return max(float(np.max(np.abs(finite), initial=0)), SYMMETRIC_LIMIT_FLOOR)


def panel_layout(count: int, params: FigureParameters) -> tuple[int, int]:
    """(rows, columns) for ``count`` panels."""
    columns = min(count, params.panels_per_row)
    return -(-count // columns), columns


def standoff_groups(poses: list[dict], params: FigureParameters) -> tuple[list[list[dict]], list[float]]:
    """Poses grouped by standoff (the board centers' sensor z) for per-standoff panels. Returns the groups, near
    to far, and their mean depths. More groups than ``max_standoff_panels`` collapse into one group (the
    standoffs are not discrete), whose depth is the mean over all."""
    depths = np.array([pose["board_center_sensor_mm"][2] for pose in poses])
    labels = group_standoffs(depths, params.standoff_gap_mm)
    if labels.max() + 1 > params.max_standoff_panels:
        labels = np.zeros_like(labels)
    groups = [[pose for pose, label in zip(poses, labels) if label == index] for index in range(labels.max() + 1)]
    return groups, [float(np.mean([p["board_center_sensor_mm"][2] for p in group])) for group in groups]


def unused_label(poses: list[dict]) -> str:
    """Legend text for the poses that took no part in the solve: "held out" when all of them were kept out by a
    plan, "rejected" when none was, else both."""
    unused = [pose for pose in poses if not pose["used"]]
    held_out = [pose for pose in unused if pose.get("held_out")]
    if unused and len(held_out) == len(unused):
        return "held out"
    return "rejected or held out" if held_out else "rejected"


def image_axes(axis, image_size_px: tuple[int, int], xlabel: str = "u (pixel)", ylabel: str = "v (pixel)") -> None:
    """Configure an axes as the image frame: pixel coordinates, v downward, equal aspect."""
    axis.set_xlim(0, image_size_px[0])
    axis.set_ylim(image_size_px[1], 0)
    axis.set_aspect("equal")
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)


# ---------------------------------------------------------------------------
# Figure 1: residuals per pose
# ---------------------------------------------------------------------------
def plot_pose_residuals(plt, block: dict, limits: dict, path: Path, params: FigureParameters) -> Path:
    """Normal and offset residual of every pose against its index, two panels (never a second axis); poses that
    took part in the solve (blue circles) and rejected poses (orange diamonds); the acceptance limit as a
    dashed 2 px line."""
    poses = [pose for pose in block["poses"] if pose["segmented"]]
    indices = np.array([i for i, pose in enumerate(block["poses"]) if pose["segmented"]])
    used = np.array([pose["used"] for pose in poses], dtype=bool)
    panels = (
        ("normal residual (deg)", np.array([pose["normal_residual_deg"] for pose in poses], dtype=float),
         (limits["maximum_normal_residual_degrees"],)),
        ("offset residual (mm)", np.array([pose["offset_residual_mm"] for pose in poses], dtype=float),
         (limits["maximum_offset_residual_mm"], -limits["maximum_offset_residual_mm"])),
    )
    with plt.rc_context(style_rc()):
        figure, axes = new_figure(plt, len(panels), 1, params)
        for axis, (ylabel, values, panel_limits) in zip(axes[:, 0], panels):
            for selected, color, marker, label in ((used, SERIES_COLOR_BLUE, MARKER_CIRCLE, "used in the solve"),
                                                   (~used, SERIES_COLOR_ORANGE, MARKER_DIAMOND, unused_label(poses))):
                if selected.any():
                    axis.scatter(indices[selected], values[selected], s=marker_area_pt2(), c=color, marker=marker,
                                 edgecolors=SURFACE_COLOR, linewidths=pixels_to_points(MARKER_EDGE_PX), zorder=3)
            for limit in panel_limits:
                axis.axhline(limit, color=INK_SECONDARY, linestyle=THRESHOLD_LINE_STYLE,
                             linewidth=pixels_to_points(LINE_WIDTH_PX), zorder=2)
            axis.set_ylabel(ylabel)
        handles = [marker_handle("used in the solve", SERIES_COLOR_BLUE, MARKER_CIRCLE)] if used.any() else []
        if not used.all():
            handles.append(marker_handle(unused_label(poses), SERIES_COLOR_ORANGE, MARKER_DIAMOND))
        add_legend(figure, handles + [line_handle("acceptance limit", INK_SECONDARY, THRESHOLD_LINE_STYLE)])
        axes[-1, 0].set_xlabel("pose index (order in the manifest)")
        axes[0, 0].set_title("Residual of every pose after registration")
        return save_figure(plt, figure, path)


# ---------------------------------------------------------------------------
# Figure 2: normal residual vector map
# ---------------------------------------------------------------------------
def plot_normal_residual_field(plt, block: dict, image_size_px: tuple[int, int], path: Path,
                               params: FigureParameters) -> Path:
    """Vector map: at each board center (u, v) an arrow for the normal residual vector R n_k - m_k in the
    sensor's image axes, one panel per standoff with a common scale; the quiver key states its length in
    degrees. Markers: the standoff's color from the Blues map (filled: used, hollow ring: rejected)."""
    poses = segmented_poses(block)
    groups, depths = standoff_groups(poses, params)
    colors = standoff_colors(plt, len(groups))
    vectors = {id(p): np.degrees(np.array(p["normal_residual_sensor"][:2], dtype=float)) for p in poses}
    longest = max([float(np.linalg.norm(v)) for v in vectors.values()] + [ARROW_LENGTH_FLOOR_DEG])
    key_degrees = nice_length(longest)
    degrees_per_pixel = longest / (params.arrow_fraction_of_width * image_size_px[0])
    rows, columns = panel_layout(len(groups), params)
    with plt.rc_context(style_rc()):
        figure, axes = new_figure(plt, rows, columns, params)
        for index, axis in enumerate(axes.ravel()):
            if index >= len(groups):
                axis.set_visible(False)
                continue
            group, color = groups[index], colors[index]
            image_axes(axis, image_size_px)
            for rejected in (False, True):
                subset = [p for p in group if (not p["used"]) == rejected]
                if not subset:
                    continue
                centers = np.array([p["board_center_uv"] for p in subset], dtype=float)
                axis.scatter(centers[:, 0], centers[:, 1], s=marker_area_pt2(),
                             facecolors=SURFACE_COLOR if rejected else color,
                             edgecolors=color if rejected else SURFACE_COLOR,
                             linewidths=pixels_to_points(MARKER_EDGE_PX), zorder=3)
            centers = np.array([p["board_center_uv"] for p in group], dtype=float)
            arrows = np.array([vectors[id(p)] for p in group])
            quiver = axis.quiver(centers[:, 0], centers[:, 1], arrows[:, 0], arrows[:, 1], angles="xy",
                                 scale_units="xy", scale=degrees_per_pixel, units="dots", width=LINE_WIDTH_PX,
                                 color=color, zorder=2)
            axis.quiverkey(quiver, *QUIVER_KEY_POSITION, key_degrees, f"{key_degrees:g} deg", labelpos="E",
                           coordinates="axes", color=INK_PRIMARY, labelcolor=INK_PRIMARY)
            axis.set_title(f"standoff {depths[index]:.0f} mm  ({len(group)} poses)", pad=TITLE_PAD_PT)
        if any(not p["used"] for p in poses):
            add_legend(figure, [marker_handle("used in the solve", INK_SECONDARY, MARKER_CIRCLE),
                                marker_handle(unused_label(poses), INK_SECONDARY, MARKER_CIRCLE, hollow=True)])
        figure.suptitle("Normal residual R n - m in the image axes (arrow length in degrees, common scale)")
        return save_figure(plt, figure, path)


# ---------------------------------------------------------------------------
# Figure 3: offset residual map
# ---------------------------------------------------------------------------
def plot_offset_residual_field(plt, block: dict, image_size_px: tuple[int, int], path: Path,
                               params: FigureParameters) -> Path:
    """At each board center a marker colored by the signed offset residual (diverging, symmetric about zero,
    common scale), one panel per standoff; rejected poses are diamonds."""
    from matplotlib.cm import ScalarMappable
    from matplotlib.colors import Normalize
    poses = segmented_poses(block)
    groups, depths = standoff_groups(poses, params)
    limit = symmetric_limit(np.array([p["offset_residual_mm"] for p in poses]))
    norm, colormap = Normalize(-limit, limit), diverging_colormap()
    rows, columns = panel_layout(len(groups), params)
    any_rejected = any(not p["used"] for p in poses)
    with plt.rc_context(style_rc()):
        figure, axes = new_figure(plt, rows, columns, params)
        for index, axis in enumerate(axes.ravel()):
            if index >= len(groups):
                axis.set_visible(False)
                continue
            image_axes(axis, image_size_px)
            for rejected, marker in ((False, MARKER_CIRCLE), (True, MARKER_DIAMOND)):
                subset = [p for p in groups[index] if (not p["used"]) == rejected]
                if not subset:
                    continue
                centers = np.array([p["board_center_uv"] for p in subset], dtype=float)
                axis.scatter(centers[:, 0], centers[:, 1], s=marker_area_pt2(),
                             c=[p["offset_residual_mm"] for p in subset], cmap=colormap, norm=norm, marker=marker,
                             edgecolors=SURFACE_COLOR, linewidths=pixels_to_points(MARKER_EDGE_PX), zorder=3)
            axis.set_title(f"standoff {depths[index]:.0f} mm  ({len(groups[index])} poses)")
        if any_rejected:
            add_legend(figure, [marker_handle("used in the solve", INK_SECONDARY, MARKER_CIRCLE),
                                marker_handle(unused_label(poses), INK_SECONDARY, MARKER_DIAMOND)])
        figure.colorbar(ScalarMappable(norm=norm, cmap=colormap), ax=axes.ravel().tolist(),
                        label="signed offset residual (mm)")
        figure.suptitle("Offset residual at the board centers")
        return save_figure(plt, figure, path)


# ---------------------------------------------------------------------------
# Figure 4: per-pixel residual maps
# ---------------------------------------------------------------------------
def pixel_residual_map(points: np.ndarray, mask: np.ndarray, plane) -> np.ndarray:
    """Signed distance (mm, positive toward the sensor) of every mask pixel's point to ``plane`` (unit normal
    toward the sensor); NaN outside the mask."""
    residual = np.full(mask.shape, np.nan)
    residual[mask] = plane.signed_distance(points[mask])
    return residual


def smoothed_gradient(residual: np.ndarray, mask: np.ndarray, sigma_px: float) -> tuple[np.ndarray, np.ndarray]:
    """Gradient (d/du, d/dv in mm per pixel) of the residual after normalized Gaussian smoothing over the mask
    (pixels outside the mask do not contribute); NaN where the smoothing has no support."""
    weight = ndimage.gaussian_filter(mask.astype(np.float64), sigma_px)
    total = ndimage.gaussian_filter(np.where(mask, residual, 0), sigma_px)
    smooth = np.where(weight > SMOOTHING_WEIGHT_FLOOR, total / np.where(weight > SMOOTHING_WEIGHT_FLOOR, weight, 1),
                      np.nan)
    d_dv, d_du = np.gradient(np.nan_to_num(smooth))
    return d_du, d_dv


def plot_pixel_residuals(plt, pose: dict, points: np.ndarray, mask: np.ndarray, plane, camera, path: Path,
                         params: FigureParameters) -> Path:
    """The per-pixel residual map of one pose (module docstring): diverging color of the signed distance with
    symmetric limits, a subsampled quiver of the smoothed in-image gradient converted to a tilt error in
    degrees (the local plane tilt the residual implies), and the mask outline."""
    from matplotlib.colors import Normalize
    residual = pixel_residual_map(points, mask, plane)
    rows, columns = np.nonzero(mask)
    row0 = max(rows.min() - params.crop_margin_px, 0)
    row1 = min(rows.max() + params.crop_margin_px, mask.shape[0] - 1)
    col0 = max(columns.min() - params.crop_margin_px, 0)
    col1 = min(columns.max() + params.crop_margin_px, mask.shape[1] - 1)
    window = (slice(row0, row1 + 1), slice(col0, col1 + 1))

    # Local tilt error: a residual slope of g mm per pixel across pixels of footprint z / f mm is the angle
    # arctan(g / footprint) between the measured surface and the predicted plane.
    d_du, d_dv = smoothed_gradient(residual, mask, params.gradient_smoothing_px)
    footprint_mm = float(np.mean(points[mask][:, 2])) / float(np.mean([camera.focal_x_px, camera.focal_y_px]))
    inner = ndimage.binary_erosion(mask, iterations=int(np.ceil(params.gradient_smoothing_px)))
    sample_rows = np.arange(row0, row1 + 1, params.quiver_step_px)
    sample_cols = np.arange(col0, col1 + 1, params.quiver_step_px)
    grid_u, grid_v = np.meshgrid(sample_cols, sample_rows)
    keep = inner[grid_v, grid_u]
    tilt_u = np.degrees(np.arctan(d_du[grid_v, grid_u] / footprint_mm))[keep]
    tilt_v = np.degrees(np.arctan(d_dv[grid_v, grid_u] / footprint_mm))[keep]
    longest = max(float(np.max(np.hypot(tilt_u, tilt_v), initial=0)), ARROW_LENGTH_FLOOR_DEG)
    key_degrees = nice_length(longest)
    degrees_per_pixel = longest / (params.arrow_fraction_of_width * (col1 - col0 + 1))
    limit = symmetric_limit(residual[mask])
    colormap = diverging_colormap().with_extremes(bad=SURFACE_COLOR)     # pixels outside the mask: the surface

    with plt.rc_context(style_rc()):
        figure, axes = new_figure(plt, 1, 1, params)
        axis = axes[0, 0]
        extent = (col0 - PIXEL_HALF_EXTENT, col1 + PIXEL_HALF_EXTENT, row1 + PIXEL_HALF_EXTENT, row0 - PIXEL_HALF_EXTENT)
        image = axis.imshow(np.ma.masked_invalid(residual[window]), cmap=colormap, norm=Normalize(-limit, limit),
                            extent=extent, interpolation="nearest")
        axis.contour(np.arange(col0, col1 + 1), np.arange(row0, row1 + 1), mask[window].astype(float),
                     levels=[MASK_CONTOUR_LEVEL], colors=INK_SECONDARY, linewidths=pixels_to_points(LINE_WIDTH_PX))
        quiver = axis.quiver(grid_u[keep], grid_v[keep], tilt_u, tilt_v, angles="xy", scale_units="xy",
                             scale=degrees_per_pixel, units="dots", width=LINE_WIDTH_PX, color=INK_PRIMARY)
        axis.quiverkey(quiver, *QUIVER_KEY_POSITION, key_degrees, f"{key_degrees:g} deg", labelpos="E",
                       coordinates="axes", color=INK_PRIMARY, labelcolor=INK_PRIMARY)
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
        axis.set_aspect("equal")
        axis.grid(False)
        axis.set_xlabel("u (pixel)")
        axis.set_ylabel("v (pixel)")
        figure.suptitle(f"pose {pose['pose_id']}, {pose['pixels']} pixels\n"
                        f"offset residual {pose['offset_residual_mm']:.3f} mm")
        figure.colorbar(image, ax=axis, label="signed distance to the predicted plane (mm)")
        return save_figure(plt, figure, path)


def choose_pixel_map_poses(block: dict, params: FigureParameters) -> list[dict]:
    """The poses with the largest |offset residual| plus a few random others (seeded), segmented ones only."""
    ranked = sorted(segmented_poses(block), key=lambda p: -abs(p["offset_residual_mm"]))
    chosen = ranked[:params.pixel_map_count]
    others = ranked[params.pixel_map_count:]
    if others and params.pixel_map_random_count > 0:
        rng = np.random.default_rng(params.seed)
        picks = rng.choice(len(others), size=min(params.pixel_map_random_count, len(others)), replace=False)
        chosen += [others[int(i)] for i in sorted(picks)]
    return chosen


def plot_pixel_maps(plt, document: dict, block: dict, masks: Mapping[str, np.ndarray], model: str,
                    pass_name: str, out_dir: Path, params: FigureParameters) -> list[Path]:
    """The per-pixel maps of the chosen poses; the capture files named by the document's manifest are read
    again. A pose whose capture or mask cannot be found is skipped with a WARNING."""
    wanted = choose_pixel_map_poses(block, params)
    if not wanted:
        return []
    pipeline = document["parameters"]["pipeline"]
    try:
        capture_set = CaptureSet(load_manifest(document["manifest"]))
        rigid, scale = transform_from_block(block)
    except (OSError, ValueError, KeyError) as error:
        print(f"WARNING: per-pixel residual maps skipped: {error}")
        return []
    written = []
    for pose in wanted:
        key = MASK_KEY_FORMAT.format(model=model, pass_name=pass_name, pose_id=pose["pose_id"])
        try:
            stack = capture_set.load_stack(pose["pose_id"])
            mask = np.asarray(masks[key], dtype=bool)
        except (OSError, ValueError, KeyError) as error:
            print(f"WARNING: pose {pose['pose_id']}: per-pixel residual map skipped ({error})")
            continue
        points = temporal_mean_points(stack.xyz, stack.valid, pipeline["min_valid_fraction"]).astype(np.float64)
        base_plane = board_plane_in_base(stack.records[0].target_pose_positioner, pipeline["target_offset_mm"])
        plane = predicted_sensor_plane(rigid, base_plane, scale).normalized(origin_on_positive_side=True)
        written.append(plot_pixel_residuals(plt, pose, points, mask, plane, stack.camera,
                                            out_dir / PIXEL_RESIDUALS_FILE_FORMAT.format(pose_id=pose["pose_id"]),
                                            params))
    return written


# ---------------------------------------------------------------------------
# Figure 5: comparison of two sessions (called by compare)
# ---------------------------------------------------------------------------
def plot_comparison(path: Path, labels: tuple[str, str], normal_residuals: tuple[np.ndarray, np.ndarray],
                    offset_residuals: tuple[np.ndarray, np.ndarray], title: str,
                    params: FigureParameters = FigureParameters()) -> Path | None:
    """Overlaid histograms of the two sessions' residuals, normal (deg) on the left and offset (mm) on the
    right (two panels, no second axis); ``title`` carries the table of RMS values. Returns None, after a
    WARNING, when matplotlib is absent."""
    plt = load_pyplot()
    if plt is None:
        return None
    from matplotlib.colors import to_rgba
    colors = (SERIES_COLOR_BLUE, SERIES_COLOR_ORANGE)
    panels = ((normal_residuals, "normal residual (deg)"), (offset_residuals, "offset residual (mm)"))
    with plt.rc_context(style_rc()):
        figure, axes = new_figure(plt, 1, len(panels), params)
        for axis, (sets, xlabel) in zip(axes[0], panels):
            pooled = np.concatenate([np.asarray(values, dtype=float) for values in sets])
            edges = np.histogram_bin_edges(pooled[np.isfinite(pooled)], bins=max(params.histogram_bins,
                                                                                HISTOGRAM_BIN_FLOOR))
            handles = []
            for values, label, color in zip(sets, labels, colors):
                axis.hist(np.asarray(values, dtype=float), bins=edges, histtype="stepfilled",
                          facecolor=to_rgba(color, HISTOGRAM_FILL_ALPHA), edgecolor=color,
                          linewidth=pixels_to_points(LINE_WIDTH_PX))
                handles.append(line_handle(label, color))
            axis.set_xlabel(xlabel)
            axis.set_ylabel("number of poses")
        add_legend(figure, handles)
        figure.suptitle(title)
        return save_figure(plt, figure, path)


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------
def make_figures(document: dict, masks: Mapping[str, np.ndarray], out_dir: Path, model: str,
                 params: FigureParameters = FigureParameters(), pass_name: str | None = None) -> list[Path]:
    """All figures of one model of a registration document (the final pass unless ``pass_name`` names one).
    Returns the files written; empty, after a WARNING, when matplotlib is absent or the pass was not solved."""
    plt = load_pyplot()
    if plt is None:
        return []
    entry = document["models"][model]
    pass_name = pass_name or entry["final_pass"]
    block = entry[pass_name]
    if block is None or not block["solved"]:
        print(f"WARNING: no figures for {model} {pass_name}: the registration was not solved")
        return []
    image_size = tuple(document["image_size_px"])
    limits = document["parameters"]["registration"]
    written = [plot_pose_residuals(plt, block, limits, out_dir / POSE_RESIDUALS_FILE, params),
               plot_normal_residual_field(plt, block, image_size, out_dir / NORMAL_FIELD_FILE, params),
               plot_offset_residual_field(plt, block, image_size, out_dir / OFFSET_FIELD_FILE, params)]
    written += plot_pixel_maps(plt, document, block, masks, model, pass_name, out_dir, params)
    return written


def build_parser() -> argparse.ArgumentParser:
    defaults = FigureParameters()
    parser = argparse.ArgumentParser(
        prog="python3 -m planereg.analysis.residual_maps",
        description="Residual figures of a registration.json (the segmentation.npz must lie next to it).")
    parser.add_argument("--registration", required=True, type=Path, help="registration.json of the register tool")
    parser.add_argument("--out", required=True, type=Path, help="output directory (created)")
    parser.add_argument("--model", default=None, help="rigid, similarity, or both (default: every model of the file)")
    parser.add_argument("--pass", dest="pass_name", choices=(PASS_ONE, PASS_TWO), default=None,
                        help="which pass to draw (default: the final one)")
    parser.add_argument("--pixel-maps", type=int, default=defaults.pixel_map_count,
                        help="per-pixel maps of the N poses with the largest |offset residual|")
    parser.add_argument("--pixel-maps-random", type=int, default=defaults.pixel_map_random_count,
                        help="per-pixel maps of this many further, randomly chosen poses")
    parser.add_argument("--seed", type=int, default=defaults.seed)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        document = load_registration(args.registration)
        masks = dict(np.load(args.registration.parent / document["segmentation_file"]))
    except (ValueError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    models = document["model_order"] if args.model in (None, "both") else [args.model]
    unknown = [m for m in models if m not in document["models"]]
    if unknown:
        print(f"ERROR: the registration has no model {unknown}; it holds {document['model_order']}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    params = FigureParameters(pixel_map_count=args.pixel_maps, pixel_map_random_count=args.pixel_maps_random,
                              seed=args.seed)
    for model in models:
        directory = args.out / model if len(models) > 1 else args.out
        written = make_figures(document, masks, directory, model, params, args.pass_name)
        print(f"wrote {len(written)} figure(s) for {model} to {directory}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
