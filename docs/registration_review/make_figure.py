"""Figure: registration error versus flange tilt range (data from verify_registration.py, check 9).

Two small multiples (rotation RMS, translation RMS) rather than a dual-axis chart.
Each panel has one series, so no legend; values are direct-labeled at the points.
Palette: the dataviz reference instance (series-1 blue, text tokens, recessive grid).
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SERIES_COLOR = "#2a78d6"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID_COLOR = "#e6e5e1"
SURFACE = "#fcfcfb"
LINE_WIDTH_PX = 2
MARKER_SIZE_PT = 7
FIGURE_DPI = 150

tbl = np.load("tilt_table.npy")                       # columns: tilt, rot RMS deg, trans RMS mm, min singular value
tilt, rot_rms, trans_rms, smin = tbl.T

fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), dpi=FIGURE_DPI, facecolor=SURFACE)
panels = [
    (axes[0], rot_rms, "Rotation error, RMS (degrees)", "{:.2f}"),
    (axes[1], trans_rms, "Translation error, RMS (mm)", "{:.2f}"),
]
for ax, y, ylabel, fmt in panels:
    ax.set_facecolor(SURFACE)
    ax.plot(tilt, y, color=SERIES_COLOR, lw=LINE_WIDTH_PX, marker="o", ms=MARKER_SIZE_PT,
            mec=SURFACE, mew=2, solid_joinstyle="round", solid_capstyle="round")
    for xt, yt in zip(tilt, y):
        ax.annotate(fmt.format(yt), (xt, yt), textcoords="offset points", xytext=(0, 9),
                    ha="center", fontsize=8, color=TEXT_SECONDARY)
    ax.set_xlabel("Flange tilt half-range about x and y (degrees)", color=TEXT_SECONDARY, fontsize=9)
    ax.set_ylabel(ylabel, color=TEXT_PRIMARY, fontsize=9)
    ax.set_xscale("log")
    ax.set_xticks(tilt)
    ax.set_xticklabels([f"±{t:.0f}°" for t in tilt], fontsize=8, color=TEXT_SECONDARY)
    ax.set_ylim(0, max(y) * 1.25)
    ax.tick_params(axis="y", labelsize=8, colors=TEXT_SECONDARY)
    ax.grid(axis="y", color=GRID_COLOR, lw=1)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(GRID_COLOR)
    ax.minorticks_off()
fig.suptitle("Registration error vs. pose diversity  (N = 17 poses, 0.2° / 0.2 mm plane noise, Section 3 method)",
             fontsize=10, color=TEXT_PRIMARY)
fig.tight_layout(rect=(0, 0, 1, 0.94))
fig.savefig("figures/error_vs_tilt.png", facecolor=SURFACE)
print("wrote figures/error_vs_tilt.png")
