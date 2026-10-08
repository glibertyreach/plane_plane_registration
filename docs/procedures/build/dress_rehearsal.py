#!/usr/bin/env python3
"""Dress rehearsal of the registration capture procedure on synthetic data, end to end.

Runs the exact chain the procedure prescribes, on a synthetic sensor and robot, at the
default plan's full scale, and records what each step reported and how long it took:

    1. the plan tool on the nominal sensor placement (section 5 of the procedure);
    2. six hand-jogged bootstrap poses (section 4), simulated, through make_manifest and the
       bootstrap tool, giving boot.json;
    3. sub-procedures A and B (section 6), simulated with clutter, fly-aways and a backlash
       emulation, 5 frames per pose, in the session layout of section 2;
    4. make_manifest and the check tool on each run (sections 7 and 8);
    5. the analysis: registration of each run against the known truth, and the A-against-B
       comparison and report.

Everything is written under --out (default: a folder in the system temporary directory);
nothing in the repository changes. The result is --out/summary.md plus the tools' own files.

    python3 docs/procedures/build/dress_rehearsal.py --out /path/to/rehearsal [--frames 5]

Run from the repository root with the planereg environment active. Expect several gigabytes
of capture files and some tens of minutes.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

# ---- The rehearsal's assumptions (every figure a named constant) ----
NOMINAL_SENSOR_TO_BASE = [[1.0, 0.0, 0.0, 800.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 1200.0], [0.0, 0.0, 0.0, 1.0]]
"""The example placement of the procedure: looking down at the workspace from 1.2 m, base z up."""
IMAGE_SIZE = ("640", "480")
FOV_DEG = ("49.9", "38.5")
"""The indicative sensor of the procedure's example plan (full horizontal and vertical field)."""
BOARD_HALF_SIZE_MM = ("100", "75")
FRAMES_PER_POSE = 5
"""Frames per pose, as section 6 of the procedure asks."""
BACKLASH_DEG = 0.03
BACKLASH_MM = 0.10
"""Backlash emulation: the order of a good industrial robot's dead band at the tool."""
BOOT_FACING_STANDOFF_MM = 600.0
BOOT_FAR_STANDOFF_MM = 850.0
BOOT_TILT_DEG = 20.0
BOOT_AZIMUTHS_DEG = (0.0, 180.0, 270.0, 90.0)
"""Section 4: facing at about 600 mm; tilted about 20 degrees top, bottom, left, right edge toward
the sensor; then about 250 mm farther away, facing."""
SEED_BOOT, SEED_A, SEED_B = 11, 1, 2


def run(log, name: str, command: list[str], cwd: Path | None = None) -> tuple[int, float, str]:
    """Run one command, log its output, return (exit code, seconds, stdout+stderr)."""
    log.write(f"\n=== {name}\n$ {' '.join(command)}\n"); log.flush()
    start = time.time()
    completed = subprocess.run(command, cwd=cwd, capture_output=True, text=True)
    seconds = time.time() - start
    output = completed.stdout + completed.stderr
    log.write(output); log.write(f"[exit {completed.returncode} in {seconds:.1f} s]\n"); log.flush()
    return completed.returncode, seconds, output


def write_boot_plan(path: Path, sensor_json: Path) -> None:
    """The six bootstrap poses of section 4, in the plan tool's poses.csv format."""
    from planereg.capture import plan_poses as pp
    from sphcal.geometry.transforms import RigidTransform
    matrix = np.asarray(json.load(open(sensor_json))["matrix"]).reshape(4, 4)
    sensor_to_base = RigidTransform(matrix[:3, :3], matrix[:3, 3])
    poses = []

    def add(pose_id, standoff, tilt, azimuth):
        center = np.array([0.0, 0.0, standoff])
        poses.append(pp.SensorPose(pose_id, standoff, tilt, azimuth, center,
                                   pp.board_rotation_in_sensor(center, tilt, azimuth)))

    add("boot01", BOOT_FACING_STANDOFF_MM, 0.0, 0.0)
    for k, azimuth in enumerate(BOOT_AZIMUTHS_DEG, start=2):
        add(f"boot{k:02d}", BOOT_FACING_STANDOFF_MM, BOOT_TILT_DEG, azimuth)
    add("boot06", BOOT_FAR_STANDOFF_MM, 0.0, 0.0)
    pp.write_poses_csv(path, poses, sensor_to_base, pp.PlanParameters())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=None, help="output folder (default: temporary)")
    parser.add_argument("--frames", type=int, default=FRAMES_PER_POSE)
    parser.add_argument("--keep-captures", action="store_true", help="keep the capture files (gigabytes)")
    args = parser.parse_args()
    out = args.out or Path(tempfile.mkdtemp(prefix="registration-rehearsal-"))
    out.mkdir(parents=True, exist_ok=True)
    py = sys.executable
    log = open(out / "log.txt", "w")
    steps: list[dict] = []

    def step(name, command, cwd=None):
        code, seconds, output = run(log, name, command, cwd)
        steps.append({"step": name, "exit": code, "seconds": round(seconds, 1)})
        print(f"{name}: exit {code} in {seconds:.0f} s", flush=True)
        return code, output

    sensor_json = out / "sensor.json"
    sensor_json.write_text(json.dumps({"matrix": list(np.asarray(NOMINAL_SENSOR_TO_BASE).reshape(-1))}))

    # 1. The plan (section 5).
    plan = out / "plan"
    step("plan", [py, "-m", "planereg.capture.plan_poses", "--sensor-in-base", str(sensor_json), "--fov-deg", *FOV_DEG,
                  "--image-size", *IMAGE_SIZE, "--board-half-size-mm", *BOARD_HALF_SIZE_MM, "--out", str(plan)])

    # 2. The bootstrap (section 4): six simulated hand-jogged captures, one frame each, no backlash.
    boot = out / "boot"
    write_boot_plan(out / "boot_plan.csv", sensor_json)
    step("simulate boot", [py, "-m", "planereg.analysis.simulate", "--out", str(boot), "--plan", str(out / "boot_plan.csv"),
                           "--sensor-to-base", str(sensor_json), "--frames", "1", "--clutter", "--flyaways",
                           "--image-size", *IMAGE_SIZE, "--fov-deg", FOV_DEG[0], "--board-half-size-mm", *BOARD_HALF_SIZE_MM,
                           "--seed", str(SEED_BOOT)])
    step("make_manifest boot", [py, "-m", "sphcal.cli.make_manifest", "--pose-log", str(boot / "pose_log.csv"),
                                "--captures", str(boot / "captures"), "--format", "csv", "--out", str(boot / "manifest.csv")])
    step("bootstrap", [py, "-m", "planereg.capture.bootstrap", "--manifest", str(boot / "manifest.csv"),
                       "--out", str(out / "boot.json")])

    # 3 and 4. Sub-procedures A and B (section 6), manifest (7) and check (8).
    for proc, seed in (("A", SEED_A), ("B", SEED_B)):
        folder = out / f"captures_{proc}"
        step(f"simulate {proc}", [py, "-m", "planereg.analysis.simulate", "--out", str(folder), "--plan", str(plan / "poses.csv"),
                                  "--sensor-to-base", str(sensor_json), "--frames", str(args.frames), "--procedure", proc,
                                  "--backlash-deg", str(BACKLASH_DEG), "--backlash-mm", str(BACKLASH_MM), "--clutter",
                                  "--flyaways", "--image-size", *IMAGE_SIZE, "--fov-deg", FOV_DEG[0],
                                  "--board-half-size-mm", *BOARD_HALF_SIZE_MM, "--seed", str(seed)])
        step(f"make_manifest {proc}", [py, "-m", "sphcal.cli.make_manifest", "--pose-log", str(folder / "pose_log.csv"),
                                       "--captures", str(folder / "captures"), "--format", "csv",
                                       "--out", str(folder / "manifest.csv")])
        step(f"check {proc}", [py, "-m", "planereg.capture.check_captures", "--manifest", str(folder / "manifest.csv"),
                               "--sensor-in-base", str(out / "boot.json"), "--pose-log", str(folder / "pose_log.csv"),
                               "--out", str(folder / "check.json")])
        # 5. The analysis.
        step(f"register {proc}", [py, "-m", "planereg.analysis.register", "--manifest", str(folder / "manifest.csv"),
                                  "--sensor-in-base", str(out / "boot.json"), "--model", "both",
                                  "--plan", str(plan / "poses.csv"),      # held-out poses stay out of the fit
                                  "--truth", str(folder / "truth.json"), "--out", str(out / f"analysis_{proc}")])
    step("compare", [py, "-m", "planereg.analysis.compare", "--registration-a", str(out / "analysis_A" / "registration.json"),
                     "--registration-b", str(out / "analysis_B" / "registration.json"), "--out", str(out / "comparison"),
                     "--label-a", "A ignoring backlash", "--label-b", "B minimizing backlash"])
    step("report", [py, "-m", "planereg.analysis.report", "--registration", str(out / "analysis_A" / "registration.json"),
                    "--comparison", str(out / "comparison" / "comparison.json"), "--out", str(out / "report_A.md")])

    # Sizes, then the capture files go unless asked to keep them.
    sizes = {p: round(sum(f.stat().st_size for f in (out / p / "captures").rglob("*")) / 1e9, 2)
             for p in ("captures_A", "captures_B") if (out / p / "captures").is_dir()}
    if not args.keep_captures:
        for p in ("captures_A", "captures_B", "boot"):
            shutil.rmtree(out / p / "captures", ignore_errors=True)
    summary = {"steps": steps, "capture_gigabytes": sizes, "frames_per_pose": args.frames,
               "backlash_deg": BACKLASH_DEG, "backlash_mm": BACKLASH_MM}
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    lines = ["# Dress rehearsal", "", "| Step | Exit | Seconds |", "|---|---|---|"]
    lines += [f"| {s['step']} | {s['exit']} | {s['seconds']} |" for s in steps]
    lines += ["", f"Capture files: {sizes} GB", f"Backlash emulation: {BACKLASH_DEG} deg, {BACKLASH_MM} mm"]
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    print(f"wrote {out / 'summary.md'}")
    return 0 if all(s["exit"] == 0 for s in steps) else 1


if __name__ == "__main__":
    sys.exit(main())
