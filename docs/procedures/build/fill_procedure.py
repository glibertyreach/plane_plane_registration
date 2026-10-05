"""Fill the generated parts of registration_capture_procedure.md from the tools themselves,
so that the document never quotes stale help text or plan numbers.

Replaces the markers {{PLAN_HELP}}, {{BOOTSTRAP_HELP}}, {{CHECK_HELP}} with the tools'
--help output, and {{PLAN_PARAGRAPH}} with the counts and predicted errors of a plan run
with the default settings on the indicative 640 x 480 sensor and a nominal sensor position;
also writes figures/fig_plan_example.png from that run. The source of truth is the
template registration_capture_procedure.template.md; this script writes the .md next to it.

Run from anywhere:  python3 docs/procedures/build/fill_procedure.py
"""
from __future__ import annotations

import json
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
PROCEDURES = HERE.parent
TEMPLATE = PROCEDURES / "registration_capture_procedure.template.md"
OUTPUT = PROCEDURES / "registration_capture_procedure.md"
FIGURE = PROCEDURES / "figures" / "fig_plan_example.png"
SUMMARY_COPY = PROCEDURES / "figures" / "plan_example_summary.txt"

# A nominal sensor position for the example plan: looking down at the workspace from 1.2 m,
# in a base frame whose z is up. Only the picture and the counts depend on it.
EXAMPLE_SENSOR_TO_BASE = [
    [1.0, 0.0, 0.0, 800.0],
    [0.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0, 1200.0],
    [0.0, 0.0, 0.0, 1.0],
]
EXAMPLE_FOV_DEG = ("49.9", "38.5")       # the indicative sensor's full field of view
EXAMPLE_IMAGE_SIZE = ("640", "480")
BOARD_HALF_SIZE_MM = ("100", "75")
HELP_BLOCK = "Output of `python3 -m planereg.capture.{tool} --help`:\n\n```\n{text}```"


def tool_help(tool: str) -> str:
    completed = subprocess.run([sys.executable, "-m", f"planereg.capture.{tool}", "--help"],
                               capture_output=True, text=True, check=True)
    return HELP_BLOCK.format(tool=tool, text=completed.stdout)


def example_plan(work: pathlib.Path) -> str:
    sensor_file = work / "sensor.json"
    sensor_file.write_text(json.dumps({"matrix": list(np.asarray(EXAMPLE_SENSOR_TO_BASE).reshape(-1))}))
    out = work / "plan"
    subprocess.run([sys.executable, "-m", "planereg.capture.plan_poses", "--sensor-in-base", str(sensor_file),
                    "--fov-deg", *EXAMPLE_FOV_DEG, "--image-size", *EXAMPLE_IMAGE_SIZE,
                    "--board-half-size-mm", *BOARD_HALF_SIZE_MM, "--out", str(out)], check=True)
    summary = (out / "plan_summary.txt").read_text()
    shutil.copy(out / "plan.png", FIGURE)
    SUMMARY_COPY.write_text(summary)
    return summary


def plan_paragraph(summary: str) -> str:
    """One paragraph quoting the plan summary verbatim, so the numbers cannot drift."""
    return ("The plan summary for these settings (reproduced from `plan_summary.txt`, "
            "and kept in `figures/plan_example_summary.txt`):\n\n```\n" + summary.rstrip() + "\n```")


def main() -> int:
    text = TEMPLATE.read_text(encoding="utf-8")
    with tempfile.TemporaryDirectory() as tmp:
        summary = example_plan(pathlib.Path(tmp))
    replacements = {
        "{{PLAN_HELP}}": tool_help("plan_poses"),
        "{{BOOTSTRAP_HELP}}": tool_help("bootstrap"),
        "{{CHECK_HELP}}": tool_help("check_captures"),
        "{{PLAN_PARAGRAPH}}": plan_paragraph(summary),
    }
    for marker, value in replacements.items():
        if marker not in text:
            raise SystemExit(f"marker {marker} missing from {TEMPLATE}")
        text = text.replace(marker, value)
    leftover = re.findall(r"\{\{[A-Z_]+\}\}", text)
    if leftover:
        raise SystemExit(f"unfilled markers: {leftover}")
    OUTPUT.write_text(text, encoding="utf-8")
    print(f"wrote {OUTPUT} and {FIGURE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
