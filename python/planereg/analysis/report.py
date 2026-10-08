"""
Markdown report of a registration (code design 8.4).

    python3 -m planereg.analysis.report --registration registration.json [--comparison comparison.json]
        --out report.md

The report holds, per transform model (the final pass of each): the verdict against the acceptance thresholds,
the transform, the residual summary, the spreads against their minimums, the per-pass numbers, the segmentation
summary (methods, pixel counts, poses whose segmentation failed, poses rejected as outliers), the per-pose
residual table (held-out poses marked), the held-out poses when ``register`` was given a plan (count, RMS and
maximum residuals against the same limits), the errors against a known truth when ``register`` was given one,
and, when a comparison is given, its RMS values and relative transform; finally the list of figures found next
to the inputs.

Only fixed sentences with the numbers filled in are produced, no free prose, so the same data always gives the
same text.

Exit codes: 0 report written, 2 an input must be fixed.

Units: millimeters, degrees.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

from planereg.analysis.compare import SCHEMA_VERSION as COMPARISON_SCHEMA_VERSION
from planereg.analysis.register import FIGURE_DIRECTORY_NAME, PASS_ONE, PASS_TWO, final_block, load_registration

EXIT_OK = 0
EXIT_INPUT_ERROR = 2
"""Exit codes: report written; an input must be fixed."""

MISSING_TEXT = "n/a"
"""Printed for a number that does not exist (NaN or null in the document)."""
DIGITS = 4
"""Decimals of the numbers in the report."""
MATRIX_DIGITS = 6
"""Decimals of the transform matrix entries."""
FIGURE_PATTERN = "*.png"
"""Figure files looked for in the figures directory next to registration.json and in comparison.json's directory."""
HELD_OUT_MARK = "held out"
"""Entry of the per-pose table's "held out" column for a pose that was kept out of the solve."""
PIXEL_PERCENTILES = (0, 50, 100)
"""Percentiles of the mask pixel counts quoted in the segmentation summary: minimum, median, maximum."""


def number(value, digits: int = DIGITS) -> str:
    """A number with fixed decimals, or MISSING_TEXT for None / NaN."""
    if value is None or not np.isfinite(value):
        return MISSING_TEXT
    return f"{value:.{digits}f}"


def verdict_word(passed: bool) -> str:
    return "within the limit" if passed else "EXCEEDS the limit"


def table(header: list[str], rows: list[list[str]]) -> list[str]:
    """A Markdown table as lines."""
    lines = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    lines += ["| " + " | ".join(row) + " |" for row in rows]
    return lines


def limit_rows(rms_normal, max_normal, rms_offset, max_offset, limits: dict) -> list[list[str]]:
    """The rows (quantity, value, limit, result) of the four residual figures against the acceptance thresholds;
    the same for the fitted poses and for the held-out poses."""
    return [
        ["RMS normal residual (deg)", number(rms_normal), number(limits["maximum_rms_normal_residual_degrees"]),
         verdict_word(rms_normal <= limits["maximum_rms_normal_residual_degrees"])],
        ["maximum normal residual (deg)", number(max_normal), number(limits["maximum_normal_residual_degrees"]),
         verdict_word(max_normal <= limits["maximum_normal_residual_degrees"])],
        ["RMS offset residual (mm)", number(rms_offset), number(limits["maximum_rms_offset_residual_mm"]),
         verdict_word(rms_offset <= limits["maximum_rms_offset_residual_mm"])],
        ["maximum offset residual (mm)", number(max_offset), number(limits["maximum_offset_residual_mm"]),
         verdict_word(max_offset <= limits["maximum_offset_residual_mm"])],
    ]


def held_out_lines(held_out: dict, limits: dict) -> list[str]:
    """The "Held-out poses" subsection of a model: what the solve did not see, judged against the fit's limits."""
    lines = ["### Held-out poses", ""]
    lines += [f"{held_out['count']} held-out poses were left out of the solve and evaluated against its transform."]
    unevaluated = held_out["unevaluated_pose_ids"]
    if unevaluated:
        lines += [f"Held-out poses that could not be evaluated (segmentation failed): {', '.join(unevaluated)}."]
    if not held_out["count"]:
        return lines + [""]
    rows = limit_rows(held_out["rms_normal_residual_deg"], held_out["max_normal_residual_deg"],
                      held_out["rms_offset_residual_mm"], held_out["max_offset_residual_mm"], limits)
    lines += ["", f"Verdict on the held-out poses: {verdict_word(held_out['held_out_within_limits'])}."]
    return lines + [""] + table(["quantity", "value", "limit", "result"], rows) + [""]


def model_section(model: str, entry: dict, limits: dict, truth_entry: dict | None) -> list[str]:
    """The report lines of one model."""
    block = final_block(entry)
    lines = [f"## Model: {model}", ""]
    verdict = "ACCEPTED" if block["accepted"] else "NOT ACCEPTED"
    lines += [f"Verdict: {verdict}. Status {block['status']}: {block['message']}. "
              f"Final pass: {entry['final_pass']}; {block['poses_used']} of {block['poses_total']} poses used.", ""]
    if not block["solved"]:
        return lines + ["No transform was solved, so there is nothing further to report for this model.", ""]

    lines += ["### Transform (sensor to base)", "", "```"]
    lines += [" ".join(f"{value:{MATRIX_DIGITS + 4}.{MATRIX_DIGITS}f}" for value in row) for row in block["matrix"]]
    lines += ["```", ""]
    lines += [f"Rotation vector {', '.join(number(v) for v in block['rotation_vector_deg'])} deg; "
              f"translation {', '.join(number(v) for v in block['translation_mm'])} mm; "
              f"scale {number(block['scale'], MATRIX_DIGITS)}.", ""]

    lines += ["### Residuals against the acceptance thresholds", ""]
    rows = limit_rows(block["rms_normal_residual_deg"], block["max_normal_residual_deg"],
                      block["rms_offset_residual_mm"], block["max_offset_residual_mm"], limits)
    lines += table(["quantity", "value", "limit", "result"], rows) + [""]

    if block.get("held_out") is not None:
        lines += held_out_lines(block["held_out"], limits)

    lines += ["### Spreads of the pose set", ""]
    spread_rows = [["normal spread", number(block["normal_spread"]), number(limits["minimum_normal_spread"]),
                    "at or above the minimum" if block["normal_spread"] >= limits["minimum_normal_spread"]
                    else "BELOW the minimum"]]
    if model == "similarity":
        spread_rows.append(["similarity spread", number(block["similarity_spread"]),
                            number(limits["minimum_similarity_spread"]),
                            "at or above the minimum" if block["similarity_spread"] >= limits["minimum_similarity_spread"]
                            else "BELOW the minimum"])
    lines += table(["quantity", "value", "minimum", "result"], spread_rows) + [""]

    lines += ["### Passes", ""]
    pass_rows = []
    for pass_name in (PASS_ONE, PASS_TWO):
        pass_block = entry[pass_name]
        if pass_block is None:
            pass_rows.append([pass_name, "not run", MISSING_TEXT, MISSING_TEXT, MISSING_TEXT])
            continue
        pass_rows.append([pass_name, f"{pass_block['status']} ({pass_block['prediction']})",
                          f"{pass_block['poses_used']}/{pass_block['poses_total']}",
                          number(pass_block["rms_normal_residual_deg"]), number(pass_block["rms_offset_residual_mm"])])
    lines += table(["pass", "status (candidate regions from)", "poses used", "RMS normal (deg)", "RMS offset (mm)"],
                   pass_rows) + [""]

    poses = block["poses"]
    methods: dict[str, int] = {}
    for pose in poses:
        if pose["segmented"]:
            methods[pose["method"]] = methods.get(pose["method"], 0) + 1
    pixels = [pose["pixels"] for pose in poses if pose["segmented"]]
    failed = [pose["pose_id"] for pose in poses if not pose["segmented"]]
    lines += ["### Segmentation", ""]
    lines += [f"Segmentation methods: {', '.join(f'{name} {count}' for name, count in sorted(methods.items())) or 'none'}."]
    if pixels:
        low, middle, high = (np.percentile(pixels, q) for q in PIXEL_PERCENTILES)
        lines += [f"Mask pixels per pose: minimum {low:.0f}, median {middle:.0f}, maximum {high:.0f}."]
    lines += [f"Poses whose segmentation failed: {', '.join(failed) if failed else 'none'}.",
              f"Poses rejected as outliers: {', '.join(block['rejected_pose_ids']) or 'none'}.", ""]

    lines += ["### Residual of every pose", ""]
    rows = [[pose["pose_id"], str(pose["method"]), str(pose["pixels"]), number(pose["plane_rms_mm"]),
             number(pose["normal_residual_deg"]), number(pose["offset_residual_mm"]),
             "yes" if pose["used"] else "no", HELD_OUT_MARK if pose.get("held_out") else ""] for pose in poses]
    lines += table(["pose", "method", "pixels", "plane RMS (mm)", "normal (deg)", "offset (mm)", "used", "held out"],
                   rows) + [""]

    if truth_entry:
        lines += ["### Errors against the known truth", ""]
        rows = [[pass_name, number(errors["rotation_error_deg"]), number(errors["translation_error_mm"]),
                 number(errors["scale"], MATRIX_DIGITS), number(errors["scale_error"], MATRIX_DIGITS)]
                for pass_name, errors in truth_entry.items() if errors is not None]
        lines += table(["pass", "rotation error (deg)", "translation error (mm)", "scale", "scale error"], rows) + [""]
    return lines


def comparison_section(comparison: dict) -> list[str]:
    """The report lines of a comparison.json."""
    labels = comparison["labels"]
    lines = ["## Comparison of two sessions", ""]
    for model in comparison["model_order"]:
        entry = comparison["models"][model]
        lines += [f"### Model: {model}", ""]
        rows = [["RMS normal residual (deg)", number(entry["rms_normal_residual_deg"]["a"]),
                 number(entry["rms_normal_residual_deg"]["b"])],
                ["RMS offset residual (mm)", number(entry["rms_offset_residual_mm"]["a"]),
                 number(entry["rms_offset_residual_mm"]["b"])],
                ["poses used", str(entry["poses_used_a"]), str(entry["poses_used_b"])]]
        lines += table(["quantity", labels["a"], labels["b"]], rows) + [""]
        held_out = entry.get("held_out")
        if held_out is not None:
            lines += table(["held-out poses", labels["a"], labels["b"]],
                           [["RMS normal residual (deg)", number(held_out["rms_normal_residual_deg"]["a"]),
                             number(held_out["rms_normal_residual_deg"]["b"])],
                            ["RMS offset residual (mm)", number(held_out["rms_offset_residual_mm"]["a"]),
                             number(held_out["rms_offset_residual_mm"]["b"])],
                            ["held-out poses", str(held_out["count"]["a"]), str(held_out["count"]["b"])]]) + [""]
        lines += [f"Session {labels['b']} has "
                  f"{'the smaller' if entry['b_smaller_rms'] else 'NOT the smaller'} residual RMS of the two "
                  f"({entry['poses_common']} poses in common)."]
        relative = entry["relative_transform"]
        if relative is not None:
            lines += [f"Relative transform between the two solutions: rotation {number(relative['rotation_angle_deg'])} deg, "
                      f"translation {number(relative['translation_distance_mm'])} mm, "
                      f"scale ratio {number(relative['scale_ratio_b_over_a'], MATRIX_DIGITS)}."]
        lines += [""]
    return lines


def figure_lines(report_directory: Path, directories: list[Path]) -> list[str]:
    """Markdown list of the PNG files found under the given directories, as paths relative to the report."""
    found = sorted({path for directory in directories if directory.is_dir() for path in directory.rglob(FIGURE_PATTERN)})
    lines = ["## Figures", ""]
    if not found:
        return lines + ["No figures were found next to the inputs.", ""]
    return lines + [f"- [{path.name}]({os.path.relpath(path, report_directory).replace(os.sep, '/')})"
                    for path in found] + [""]


def build_report(document: dict, comparison: dict | None, registration_path: Path, report_directory: Path,
                 comparison_path: Path | None) -> str:
    """The report text."""
    limits = document["parameters"]["registration"]
    lines = ["# Registration report", "",
             f"Manifest: {document['manifest']}", "",
             f"Rough transform for the first pass: {document['sensor_in_base'] or 'none (closest large plane)'}.",
             f"Outlier rejection rounds: {limits['outlier_rejection_rounds']}."]
    plan = document.get("plan")
    if plan is not None:
        lines += [f"Pose plan (held-out poses): {plan['path']}; {len(plan['poses_not_captured'])} of "
                  f"{plan['poses_planned']} plan poses were not captured, "
                  f"{len(plan['manifest_poses_not_in_plan'])} manifest poses are not in the plan."]
    lines += [""]
    truth = document.get("truth_comparison") or {}
    for model in document["model_order"]:
        lines += model_section(model, document["models"][model], limits, truth.get(model))
    if comparison is not None:
        lines += comparison_section(comparison)
    directories = [registration_path.parent / FIGURE_DIRECTORY_NAME]
    if comparison_path is not None:
        directories.append(comparison_path.parent)
    lines += figure_lines(report_directory, directories)
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python3 -m planereg.analysis.report",
        description="Write a Markdown report of a registration (and optionally a comparison).")
    parser.add_argument("--registration", required=True, type=Path, help="registration.json of the register tool")
    parser.add_argument("--comparison", type=Path, default=None, help="comparison.json of the compare tool")
    parser.add_argument("--out", required=True, type=Path, help="report file to write (report.md)")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        document = load_registration(args.registration)
        comparison = None
        if args.comparison is not None:
            try:
                comparison = json.loads(args.comparison.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as error:
                raise ValueError(f"cannot read {args.comparison}: {error}") from None
            if comparison.get("schema_version") != COMPARISON_SCHEMA_VERSION:
                raise ValueError(f"{args.comparison} is not a comparison.json of schema version "
                                 f"{COMPARISON_SCHEMA_VERSION}")
    except ValueError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    args.out.parent.mkdir(parents=True, exist_ok=True)
    text = build_report(document, comparison, args.registration, args.out.parent.resolve(), args.comparison)
    args.out.write_text(text, encoding="utf-8")
    print(f"wrote {args.out}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
