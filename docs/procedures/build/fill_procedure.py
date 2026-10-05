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
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap

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


HELP_COLUMNS = "76"
"""Terminal width given to argparse when the help text is captured, so that every line of
the quoted help fits the Word page at the code font size of build/reference.docx."""
SUMMARY_WRAP_COLUMNS = 84
"""Lines of the plan summary longer than this are wrapped (with their indent kept)."""


def tool_help(tool: str) -> str:
    environment = {**os.environ, "COLUMNS": HELP_COLUMNS}
    completed = subprocess.run([sys.executable, "-m", f"planereg.capture.{tool}", "--help"],
                               capture_output=True, text=True, check=True, env=environment)
    return HELP_BLOCK.format(tool=tool, text=completed.stdout)


def wrap_summary(summary: str) -> str:
    """Wrap the plan summary's long sentences so the quoted block fits the page; table rows
    and short lines pass through unchanged."""
    wrapped = []
    for line in summary.splitlines():
        indent = len(line) - len(line.lstrip(" "))
        if len(line) <= SUMMARY_WRAP_COLUMNS:
            wrapped.append(line)
        else:
            wrapped.extend(textwrap.wrap(line, SUMMARY_WRAP_COLUMNS, initial_indent=" " * indent,
                                         subsequent_indent=" " * (indent + 2)))
    return "\n".join(wrapped)


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
    """The example plan summary with an explanation of where it comes from and how to read it.
    The summary is quoted verbatim from the tool, so its numbers cannot drift from the code."""
    # The example was run from a temporary file; name the source generically instead.
    summary = re.sub(r"matrix from \S+sensor\.json", "matrix from boot.json", summary)
    lead = (
        "What the plan tool prints. Besides the three files, the tool prints a plain-text\n"
        "summary to the screen and saves the same text as `plan/plan_summary.txt`. It is not\n"
        "something you fill in; it is the tool's report on the plan it just made. The block\n"
        "below is that report for the default settings, a 640 x 480 sensor and a nominal\n"
        "sensor position (your own run shows the same layout with your numbers). Read it\n"
        "before handing the plan to the robot programmer:\n\n"
        "- The table gives, for every standoff and tilt, how many poses were planned, how\n"
        "  many were dropped because a board corner would fall too near the image edge, how\n"
        "  many are kept, and how many of those are tagged held out. A few dropped poses are\n"
        "  normal; if a whole row is dropped, the board is too large for that standoff in\n"
        "  this sensor's field, and the line after the table says which option to change.\n"
        "- `normal spread` says how well the tilts cover all directions; the registration\n"
        "  refuses a set below 0.05. The default plan gives about 0.28. `similarity spread`\n"
        "  matters only for the optional scale analysis and must exceed 0.01.\n"
        "- The `predicted ... RMS error` lines are the registration error to expect if the\n"
        "  sensor's planes are good to 0.2 degrees and 0.2 mm; they are an order of\n"
        "  magnitude, not a promise. The second quality block repeats the numbers for the\n"
        "  poses that are not held out, which is the set the engineer fits.\n"
        "- The last two lines are the size of the job: the number of poses, and the number of\n"
        "  pose visits and capture files once both sub-procedures (section 6) have been run.\n"
        "  Check them against the time and disk space available before starting.\n\n"
        "Send `plan_summary.txt` to the engineer with the other deliverables (section 9). The\n"
        "example below is also kept as `figures/plan_example_summary.txt`.\n\n"
    )
    return lead + "```\n" + wrap_summary(summary.rstrip()) + "\n```"


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
