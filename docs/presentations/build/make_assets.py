"""Make the figure panels used by the registration decks from the procedure's figures.

Run from the repository root:
    python3 docs/presentations/build/make_assets.py

Reads  docs/procedures/figures/fig_*.png
Writes docs/presentations/assets/*.png

Each panel is cut out of its source figure by a named box, its panel title is erased, and the
remaining content is cropped to its own bounds with a fixed margin restored on every side (so that
removing a title leaves no gap at the top and no title remains). Margins are filled with the source
figure's own background color (pure white for fig_mounting and fig_plan_example, a very light
off-white for fig_approach and fig_residuals), so the margin does not show as a tinted frame.
All numbers below are pixels of the source figures, found by inspecting them.
"""
from pathlib import Path

import numpy as np
from PIL import Image

REPO_ROOT = Path(__file__).resolve().parents[3]  # build/ -> presentations/ -> docs/ -> repository root
FIGURES_DIR = REPO_ROOT / "docs" / "procedures" / "figures"
ASSETS_DIR = REPO_ROOT / "docs" / "presentations" / "assets"

MARGIN_PX = 20  # white margin restored on every side of a cropped panel
CONTENT_THRESHOLD = 12  # a pixel is content when its RGB differs from the background by more than this (sum of channel differences)

# fig_mounting.png (2400 x 700): panel (a) on the left, panel (b) on the right.
MOUNTING_SPLIT_X = 1261  # blank column between the panels (blank from x = 1256 to 1266): (a) is left of it, (b) right of it
MOUNTING_A_TITLE_BOX = (0, 70, MOUNTING_SPLIT_X, 112)  # (x0, y0, x1, y1) of title "(a) ..."; content starts at y = 119 below it
MOUNTING_B_TITLE_BOX = (MOUNTING_SPLIT_X, 105, 2400, 150)  # title "(b) ..."; content starts at y = 158 below it

# fig_approach.png (1350 x 600): sub-procedure A on the left, sub-procedure B on the right.
APPROACH_SPLIT_X = 675  # blank columns 653 to 696 lie between the two axes frames
APPROACH_B_TITLE_BOX = (APPROACH_SPLIT_X, 0, 1350, 25)  # title "B minimizing backlash" is at y = 1 to 21; the axes frame starts at y = 29

# fig_residuals.png (780 x 540): one panel with its title above the axes frame.
RESIDUALS_TITLE_BOX = (0, 0, 780, 46)  # title "The two residuals ..." is at y = 22 to 42; the axes frame starts at y = 50


def background(img):
    """Background color of a figure: its top-left pixel (every source figure has an opaque background)."""
    return tuple(img.getpixel((0, 0))[:3])


def panel(img, x_range, title_box):
    """Cut a panel out of a figure: erase its title box, crop to the content bounds, restore the margins."""
    rgb = img.convert("RGB")
    bg = background(rgb)
    x0, x1 = x_range
    work = rgb.crop((x0, 0, x1, rgb.height))
    if title_box is not None:  # title_box is in source coordinates; shift it into the panel's
        tx0, ty0, tx1, ty1 = title_box
        work.paste(bg, (tx0 - x0, ty0, tx1 - x0, ty1))
    diff = np.abs(np.array(work).astype(int) - np.array(bg)).sum(axis=2) > CONTENT_THRESHOLD
    ys = np.where(diff.any(axis=1))[0]
    xs = np.where(diff.any(axis=0))[0]
    content = work.crop((int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1))
    out = Image.new("RGB", (content.width + 2 * MARGIN_PX, content.height + 2 * MARGIN_PX), bg)
    out.paste(content, (MARGIN_PX, MARGIN_PX))
    return out


def main():
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    mounting = Image.open(FIGURES_DIR / "fig_mounting.png")
    approach = Image.open(FIGURES_DIR / "fig_approach.png")
    residuals = Image.open(FIGURES_DIR / "fig_residuals.png")
    outputs = {
        "mounting_a_side.png": panel(mounting, (0, MOUNTING_SPLIT_X), MOUNTING_A_TITLE_BOX),
        "mounting_b_front.png": panel(mounting, (MOUNTING_SPLIT_X, mounting.width), MOUNTING_B_TITLE_BOX),
        "approach_b.png": panel(approach, (APPROACH_SPLIT_X, approach.width), APPROACH_B_TITLE_BOX),
        "residuals.png": panel(residuals, (0, residuals.width), RESIDUALS_TITLE_BOX),
        "plan_example.png": Image.open(FIGURES_DIR / "fig_plan_example.png").convert("RGB"),  # used whole
    }
    for name, img in outputs.items():
        img.save(ASSETS_DIR / name, optimize=True)
        print(f"{name}: {img.width} x {img.height}")


if __name__ == "__main__":
    main()
