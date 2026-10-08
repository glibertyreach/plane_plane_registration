"""
Comparison of two registered sessions of the same plan (sub-procedure A against B, code design 8.3).

    python3 -m planereg.analysis.compare --registration-a A/registration.json
        --registration-b B/registration.json --out DIR [--model rigid|similarity] [--label-a A] [--label-b B]

For every transform model the two registrations have in common (the final pass of each):
    * the per-pose residual differences (B minus A) over the poses that took part in both solves;
    * the RMS normal and offset residual of each session over the poses that took part in its own solve, and
      over the common poses;
    * when both registrations have held-out poses (register ``--plan``), the RMS normal and offset residual of
      each session's own held-out poses and their counts (they are not compared pose by pose: the two sessions
      need not hold out the same poses);
    * the relative transform between the two solutions: the rotation angle between their rotations (deg), the
      distance between their translations (mm), and the ratio of their scales. A constant backlash is absorbed
      by the registration as a constant shift of the sensor placement, so procedure B shows a small residual
      RMS and, against a known truth, a shifted transform; this relative transform is how the two differ.

Outputs in --out: ``comparison.json`` (all models) and ``comparison.png`` (overlaid histograms of the two
residual sets of the primary model, the RMS values in the title; see residual_maps.plot_comparison). The
primary model is --model, else rigid when both sessions solved it, else the first model they share.

Exit codes: 0 comparison written, 2 an input must be fixed.

Units: millimeters, degrees.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from planereg.analysis import residual_maps
from planereg.analysis.register import (MODEL_RIGID, final_block, json_safe, load_registration,
                                        transform_from_block)

EXIT_OK = 0
EXIT_INPUT_ERROR = 2
"""Exit codes: comparison written; an input must be fixed."""
SCHEMA_VERSION = 1
"""Version of the comparison.json layout (report.py checks it)."""
HELD_OUT_KEYS = ("rms_normal_residual_deg", "rms_offset_residual_mm", "count")
"""Keys of the held-out section of registration.json that comparison.json carries for both sessions."""
COMPARISON_FILE_NAME = "comparison.json"
"""Name of the JSON written into --out (the figure is residual_maps.COMPARISON_FILE)."""


def used_residuals(block: dict) -> dict[str, tuple[float, float]]:
    """{pose_id: (normal residual deg, offset residual mm)} of the poses that took part in a solve."""
    return {pose["pose_id"]: (pose["normal_residual_deg"], pose["offset_residual_mm"])
            for pose in block["poses"] if pose["used"]}


def rms(values: list[float] | np.ndarray) -> float:
    """Root mean square; NaN for an empty list."""
    array = np.asarray(values, dtype=np.float64)
    return float(np.sqrt(np.mean(array ** 2))) if array.size else float("nan")


def compare_model(block_a: dict, block_b: dict) -> dict:
    """The comparison of two result blocks of the same model (module docstring)."""
    residuals_a, residuals_b = used_residuals(block_a), used_residuals(block_b)
    common = [pose_id for pose_id in residuals_a if pose_id in residuals_b]
    entry = {
        "poses_used_a": len(residuals_a), "poses_used_b": len(residuals_b), "poses_common": len(common),
        "status_a": block_a["status"], "status_b": block_b["status"],
        "rms_normal_residual_deg": {"a": rms([v[0] for v in residuals_a.values()]),
                                    "b": rms([v[0] for v in residuals_b.values()])},
        "rms_offset_residual_mm": {"a": rms([v[1] for v in residuals_a.values()]),
                                   "b": rms([v[1] for v in residuals_b.values()])},
        "rms_normal_residual_deg_common": {"a": rms([residuals_a[p][0] for p in common]),
                                           "b": rms([residuals_b[p][0] for p in common])},
        "rms_offset_residual_mm_common": {"a": rms([residuals_a[p][1] for p in common]),
                                          "b": rms([residuals_b[p][1] for p in common])},
        "pose_differences": [{"pose_id": p,
                              "normal_residual_deg_a": residuals_a[p][0], "normal_residual_deg_b": residuals_b[p][0],
                              "offset_residual_mm_a": residuals_a[p][1], "offset_residual_mm_b": residuals_b[p][1],
                              "normal_difference_deg": residuals_b[p][0] - residuals_a[p][0],
                              "offset_difference_mm": residuals_b[p][1] - residuals_a[p][1]} for p in common],
        "relative_transform": None,
    }
    entry["b_smaller_rms"] = bool(
        entry["rms_normal_residual_deg"]["b"] < entry["rms_normal_residual_deg"]["a"]
        and entry["rms_offset_residual_mm"]["b"] < entry["rms_offset_residual_mm"]["a"])
    held_out_a, held_out_b = block_a.get("held_out"), block_b.get("held_out")
    if held_out_a is not None and held_out_b is not None:
        entry["held_out"] = {key: {"a": held_out_a[key], "b": held_out_b[key]} for key in HELD_OUT_KEYS}
    if block_a["solved"] and block_b["solved"]:
        rigid_a, scale_a = transform_from_block(block_a)
        rigid_b, scale_b = transform_from_block(block_b)
        translation_distance, rotation_angle = rigid_a.difference_from(rigid_b)
        entry["relative_transform"] = {"rotation_angle_deg": rotation_angle,
                                       "translation_distance_mm": translation_distance,
                                       "scale_ratio_b_over_a": scale_b / scale_a}
    return entry


def choose_primary(shared: list[str], requested: str | None) -> str | None:
    """The model whose histograms are drawn: the requested one, else rigid, else the first shared."""
    if requested is not None:
        return requested if requested in shared else None
    return MODEL_RIGID if MODEL_RIGID in shared else (shared[0] if shared else None)


def figure_title(labels: tuple[str, str], entry: dict) -> str:
    """Title of comparison.png: the RMS values of both sessions (the design's 'table of RMS values')."""
    lines = []
    for key, label in (("a", labels[0]), ("b", labels[1])):
        lines.append(f"{label}: RMS normal {entry['rms_normal_residual_deg'][key]:.3f} deg, "
                     f"RMS offset {entry['rms_offset_residual_mm'][key]:.3f} mm "
                     f"({entry['poses_used_' + key]} poses)")
    relative = entry["relative_transform"]
    if relative is not None:
        lines.append(f"relative transform: {relative['rotation_angle_deg']:.3f} deg, "
                     f"{relative['translation_distance_mm']:.3f} mm")
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python3 -m planereg.analysis.compare",
        description="Compare the registrations of two sessions of the same plan (sub-procedures A and B).")
    parser.add_argument("--registration-a", required=True, type=Path, help="registration.json of session A")
    parser.add_argument("--registration-b", required=True, type=Path, help="registration.json of session B")
    parser.add_argument("--out", required=True, type=Path, help="output directory (created)")
    parser.add_argument("--model", default=None, help="model drawn in comparison.png (default: rigid, else the first shared)")
    parser.add_argument("--label-a", default="A", help="name of session A in the figure and the report")
    parser.add_argument("--label-b", default="B", help="name of session B in the figure and the report")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        document_a = load_registration(args.registration_a)
        document_b = load_registration(args.registration_b)
    except ValueError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    shared = [model for model in document_a["model_order"] if model in document_b["models"]]
    primary = choose_primary(shared, args.model)
    if primary is None:
        print(f"ERROR: the sessions share no transform model (A has {document_a['model_order']}, "
              f"B has {document_b['model_order']}"
              + ("" if args.model is None else f"; --model {args.model} is not shared") + ")", file=sys.stderr)
        return EXIT_INPUT_ERROR
    labels = (args.label_a, args.label_b)
    models = {model: compare_model(final_block(document_a["models"][model]), final_block(document_b["models"][model]))
              for model in shared}
    args.out.mkdir(parents=True, exist_ok=True)
    figure_path = None
    block_a, block_b = (final_block(document["models"][primary]) for document in (document_a, document_b))
    figure = residual_maps.plot_comparison(
        args.out / residual_maps.COMPARISON_FILE, labels,
        tuple(np.array([p["normal_residual_deg"] for p in block["poses"] if p["used"]], dtype=float)
              for block in (block_a, block_b)),
        tuple(np.array([p["offset_residual_mm"] for p in block["poses"] if p["used"]], dtype=float)
              for block in (block_a, block_b)),
        figure_title(labels, models[primary]))
    if figure is not None:
        figure_path = residual_maps.COMPARISON_FILE
    document = json_safe({
        "schema_version": SCHEMA_VERSION,
        "registration_a": args.registration_a.resolve(), "registration_b": args.registration_b.resolve(),
        "labels": {"a": labels[0], "b": labels[1]},
        "primary_model": primary, "model_order": shared, "models": models, "figure": figure_path,
    })
    (args.out / COMPARISON_FILE_NAME).write_text(json.dumps(document, indent=2), encoding="utf-8")
    for model, entry in models.items():
        relative = entry["relative_transform"]
        print(f"{model}: RMS normal {entry['rms_normal_residual_deg']['a']:.4f} ({labels[0]}) vs "
              f"{entry['rms_normal_residual_deg']['b']:.4f} ({labels[1]}) deg; RMS offset "
              f"{entry['rms_offset_residual_mm']['a']:.4f} vs {entry['rms_offset_residual_mm']['b']:.4f} mm"
              + ("" if relative is None else f"; relative transform {relative['rotation_angle_deg']:.4f} deg, "
                                              f"{relative['translation_distance_mm']:.4f} mm"))
    print(f"wrote {args.out / COMPARISON_FILE_NAME}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
