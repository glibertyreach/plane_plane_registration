# planereg: code design for gathering and analyzing the registration sample set

Status: interfaces fixed for implementation, 2026-10-04. This is the contract the
`planereg` package (`python/planereg/`) is written against. The method it serves is
the plane-plane registration of `docs/registration_review/REVIEW.md` (Section 6 is the
algorithm; Section 9 the author's decisions); the capture procedure it supports is
`docs/procedures/registration_capture_procedure.md`, modeled on the stage-1 procedure
of the sibling repository `depth_calibration_from_spherical_target`.

Terminology (author's convention): *registration* is the determination of the
sensor-to-base transform (extrinsics); *calibration* is the determination of sensor
intrinsics. The word "calibration" appears here only in the names of sphcal modules.

## 1. Scope and non-goals

In scope: planning the robot poses of a registration session, including the two
sub-procedures that differ in how each pose is approached (Section 6); the
bootstrap that finds the sensor roughly; the quick check of a capture set;
segmentation of the target board in a capture, with and without a prediction of
where it is; the registration itself (rigid and similarity) with per-pose residuals;
residual maps, including vector maps; a report; a synthetic session generator for
tests.

Out of scope: the robot program itself (the procedure gives its outline); reading
or writing capture files (sphcal does that); sensor intrinsic calibration; the C++
library (the Python solver here is its reference, and both follow REVIEW.md).

## 2. Coding rules

Identical to sphcal's (`docs/design/code_design.md`, Section 2, of that repository):
Python 3.10+, numpy, scipy (matplotlib for figures, pytest for tests); clarity over
terseness; every module and every non-trivial function documented with what it
computes, in which frame and units; no magic numbers (every constant is a named
parameter of a dataclass or a documented module constant, numerical guards included);
millimeters, degrees at public interfaces, pixels for image coordinates; image arrays
(height, width[, channels]) with pixel (u, v) = (column, row); invalid depth is
z <= 0 (the whole point zero) in a file, NaN after temporal averaging; U.S. spelling.

## 3. Dependencies and layout

```
python/
  pyproject.toml           distribution "planereg"; depends on numpy, scipy, sphcal
  planereg/
    core/                  shared mathematics (done): planes.py, registration.py, segmentation.py
    capture/               the gathering tools (to implement, Section 7)
    analysis/              the analysis tools (to implement, Section 8)
  tests/                   pytest; synthetic data only
```

`sphcal` is installed from the sibling repository (`pip install -e <path>`). It
supplies, and planereg must reuse rather than copy:

| sphcal module | What planereg takes from it |
|---|---|
| `sphcal.io.matcloud` | `read_matcloud`, `write_matcloud` (the verified `.mc` reader/writer) |
| `sphcal.io.poses` | `CaptureRecord`, `load_manifest`, `write_manifest_csv/json`, `TARGET_KIND_BOARD` |
| `sphcal.io.capture_set` | `CaptureSet` (records grouped by pose, lazy `PoseStack`) |
| `sphcal.cli.make_manifest` | the pose-log reader and the `make_manifest` tool, used unchanged by the technician |
| `sphcal.cli.plan_poses` | `POSE_CSV_COLUMNS`, `frame_from_z_axis`, `board_corners_in_view`, `load_sensor_in_base` (reuse what fits; do not call its sphere planning) |
| `sphcal.geometry.camera` | `PinholeCamera` |
| `sphcal.geometry.transforms` | `RigidTransform` (R @ x + t; `compose(a, b)` applies b first) |
| `sphcal.features.depth_features` | `temporal_mean_points` (frame averaging) |
| `sphcal.simulate.synthetic` | `SyntheticSensorParameters`, `render_board_frame` (synthetic boards) |

`capture` and `analysis` import `core` and sphcal; they never import each other.

## 4. Frames, planes, conventions

- S: sensor frame (frame of the points in a capture). B: robot base frame. T: board
  tool frame, origin at the center of the board's front face, z out of the face toward
  the sensor, x along the long edge (stage-1 procedure, Section 4). The manifest's
  target pose is T -> B (sphcal `CaptureRecord.target_pose_positioner`).
- The board's front-face plane in T is (0, 0, 1, 0). `board_plane_in_base` carries it
  to B; its `target_offset_mm` is 0 for a logged tool frame and D for a logged flange
  frame (spec Eq. 3).
- A sensor-frame plane is stored normalized with its normal toward the sensor
  (offset > 0). `Plane.normalized()` does this; `Plane.unit()` scales without flipping.
- The registration result is X: S -> B, `RegistrationResult.sensor_to_base` (R, s) and
  `scale` c, with `sensor_to_base_matrix()` = [cR s; 0 1].
- A rough transform ("bootstrap") is the same object, written as JSON
  `{"matrix": [16 floats, row-major]}`, the format sphcal's plan tool reads.

## 5. core (implemented)

`core/planes.py`: `Plane` (normal, offset; `normalized`, `unit`, `signed_distance`,
`angle_to_degrees`), `transform_plane` (inverse transpose in closed form),
`board_plane_in_base`, `predicted_sensor_plane` (X^t applied to a base plane, scaled).

`core/registration.py`: `RegistrationParameters` (names and defaults identical to the
C++), `TransformModel`, `RegistrationStatus`, `register_planes(sensor_planes,
base_planes, params) -> RegistrationResult` (per-pose residuals, spreads, RMS and max,
drop-worst outlier rejection), `solve_rotation_between_directions`, `spread`,
`expected_registration_error(normal_spread, N, normal_noise_deg, offset_noise_mm)`.

`core/segmentation.py`: `SegmentationParameters`, `PlanePrediction`,
`board_prediction(sensor_to_base_rough, board_to_base, half_size_mm)`,
`segment_target_plane(points, camera, params, prediction=None) -> SegmentationResult`
(mask, plane, rms, inlier count, method, candidate areas), with the building blocks
`fit_plane_pca`, `closest_large_plane`, `predicted_region`, `refine_plane`,
`largest_component`, `pixel_footprint_mm2`. See the module docstring for the method.

Per-pose pipeline shared by the capture check and the analysis: `CaptureSet.load_stack`
-> `temporal_mean_points(stack.xyz, stack.valid, min_valid_fraction)` ->
`segment_target_plane` -> `board_plane_in_base(record.target_pose_positioner)` ->
`register_planes` over the poses. Put this pipeline in **one** place,
`core/pipeline.py` (to implement, Section 7.0), so both tools segment identically.

## 6. The two sub-procedures and the pose budget

Every planned pose carries a *target pose* and an *approach pose*. The approach pose
is the target pose retreated by `approach_retreat_mm` along the board's own -z (away
from the sensor, along its normal) and rotated by `approach_rotation_deg` about the
board's x axis, both fixed for the whole plan, so that the final move onto every
target is the same displacement in the tool frame.

- **Sub-procedure A, "ignoring backlash"**: the robot moves to each target pose
  directly (one joint move), settles, captures. The approach pose is ignored.
- **Sub-procedure B, "minimizing backlash"**: the robot moves to the approach pose
  (joint move), then makes a linear move onto the target pose, settles, captures. The
  intent is that every joint's last motion has the same sense at every pose, so gear
  backlash is taken up the same way each time. A fixed Cartesian final move makes this
  true within a region of the workspace, not globally; the pose log therefore has
  optional joint-angle columns (`j1`..`j6` at the target, `approach_j1`..`approach_j6`
  at the approach pose) and the check tool reports, per joint, the fraction of poses
  whose final joint motion has the majority sign. The procedure asks for the
  controller's backlash compensation to be left as in production.

Both sub-procedures run the same `poses.csv`, into separate capture folders
(`captures_A/`, `captures_B/`) with separate pose logs and manifests; the analysis
registers each and compares them. The pose ids are the same in both.

Pose budget (author's decision): a first run of a few hundred poses per
sub-procedure (fewer than 500); the plan tool reports the predicted registration
error of the plan, and a run above 1000 poses is planned only if the first run's
residual analysis calls for it. The plan tool's defaults produce about 240 poses
with the default board and field (3 standoffs x 9 lateral positions x 9
orientations, minus the few that leave the image); `--target-pose-count` scales the
grid density to hit a requested count approximately.

## 7. capture (to implement)

All tools are `python3 -m planereg.capture.<tool>`, argparse, `main(argv) -> int`,
exit codes 0 ok / 1 flagged / 2 input error, like sphcal's. Messages to the technician
start with `ERROR:` or `WARNING:` and name the pose or file.

### 7.0 `core/pipeline.py` (shared, implement first)

```
@dataclass(frozen=True) PipelineParameters:
    min_valid_fraction: float = 0.5          # temporal_mean_points
    segmentation: SegmentationParameters
    target_offset_mm: float = 0.0            # board_plane_in_base
@dataclass PoseMeasurement:
    pose_id, kind, frames, valid_fraction (as sphcal's temporal_valid_fraction),
    segmentation: SegmentationResult, sensor_plane: Plane | None, base_plane: Plane,
    board_center_uv: (u, v) of the mask's centroid pixel, board_center_sensor_mm (3,),
    touches_border: bool  (the mask BEFORE erosion, not all valid pixels, within border_margin_px of the border)
def measure_pose(capture_set, pose_id, params, rough_sensor_to_base=None) -> PoseMeasurement
def measure_all(capture_set, params, rough_sensor_to_base=None) -> list[PoseMeasurement]
    # with a rough transform: prediction per pose; without: closest large plane
def register_measurements(measurements, registration_params) -> (RegistrationResult, used_pose_ids)
    # skips failed segmentations; the ids are in the order of the result's residuals
```

### 7.1 `capture/plan_poses.py`

Inputs: `--sensor-in-base PATH` (JSON matrix or bootstrap, as sphcal), `--camera
PATH` (.mc) or `--fov-deg H V --image-size W H`, `--board-half-size-mm HW HH`
(default 100 75), standoffs `--standoffs-mm` (default 550 750 950; at 450 mm the
board fills so much of the field that most tilted poses leave the image), tilts
`--tilts-deg` (default 0 15 30), azimuths `--azimuths-deg` (default 0 90 180 270;
tilt 0 is planned once per lateral position), `--lateral-fill` (default 0.5: fraction
of the half field covered by board centers at each standoff), `--lateral-positions`
(default 3 x 3 grid), `--edge-margin-px` (default 10), `--approach-retreat-mm`
(default 40), `--approach-rotation-deg` (default 3), `--holdout-fraction` (0.2; the analysis tool reads the plan with `--plan` and keeps the tagged poses out of the fit, reporting their residuals as the independent check),
`--seed`, `--target-pose-count` (optional; scales the lateral grid), `--noise-normal-deg`
(0.2) and `--noise-offset-mm` (0.2) for the error prediction, `--out DIR`.

Board orientation at a pose: in S, z points from the board center toward the sensor
origin (facing), then tilted by the tilt angle about the in-plane axis at the given
azimuth; x from `frame_from_z_axis`. Poses whose four corners (after the tilt) fall
within the edge margin of the image are dropped and counted. Poses are carried to B
with the rough transform.

Outputs in `--out`: `poses.csv` with sphcal's `POSE_CSV_COLUMNS` (so
`sphcal.cli.make_manifest` accepts it as a pose log) **plus** the columns
`standoff_mm, tilt_deg, azimuth_deg, approach_x_mm, approach_y_mm, approach_z_mm,
approach_r00..approach_r22, approach_quat_w..approach_quat_z,
approach_rotvec_x_deg..approach_rotvec_z_deg` (the approach pose in B: row-major
rotation, unit quaternion with w >= 0, and rotation vector in degrees, the same three
forms as the target pose);
`plan_summary.txt` (counts per standoff and tilt, dropped poses, the plan's normal
spread, its similarity spread, the predicted rotation and translation RMS error from
`expected_registration_error`, and a line saying how many poses the two sub-procedures
make together); `plan.png` (two panels as sphcal's: side and front views of the board
centers with the field of view, boards drawn as their outlines, held-out ones
distinguished; one hue per standoff, legend present).

### 7.2 `capture/bootstrap.py`

Inputs: `--manifest PATH` (made by `make_manifest` from 4 or more hand-jogged board
captures), `--out PATH` (JSON), registration thresholds as options, and
`--target-offset-mm D` (0 for a logged board tool frame, the flange-face-to-front-face distance D when the
flange pose was logged; it reaches `PipelineParameters.target_offset_mm`). Runs
`measure_all` without a prediction, `register_measurements` (rigid), prints a table
(pose, method, pixels, plane RMS, normal residual, offset residual) and the
transform, writes `{"matrix": [...], "residuals": {...}, "normal_spread": ...}`. Exit
1 if the status is not Success, with advice (tilt more, re-jog a pose, check the
tool frame).

### 7.3 `capture/check_captures.py`

Inputs: `--manifest`, `--sensor-in-base PATH` (optional; enables the predicted region),
`--pose-log PATH` (optional; for the joint-sign report), thresholds (segmentation RMS
warn 1.0 mm, min valid fraction 0.5, border margin 4 px, normal residual warn 1.0
deg, offset residual warn 1.0 mm), `--target-offset-mm D` as in the bootstrap,
`--min-mask-pixels N` (default 1000: a board image with fewer segmented pixels is
flagged as too small for a reliable plane and its residual flags are suppressed; the
advice is to drop the pose from the plan), `--out PATH`.

Per pose: frames, valid fraction, segmentation method and pixels, plane RMS, border
contact of the mask, flags (unreadable, low valid, border, segmentation failed, plane
RMS). Then `register_measurements` over the successful poses; per pose the normal and
offset residuals with flags; the normal spread and the similarity spread of the set
with a warning below the registration defaults; the joint-sign report when joint
columns exist. Table, verdict line, JSON report, exit codes as sphcal.

## 8. analysis (to implement)

### 8.1 `analysis/register.py`

`python3 -m planereg.analysis.register --manifest PATH --out DIR [--sensor-in-base PATH]
[--model rigid|similarity|both] [--outlier-rounds N] [--target-offset-mm D]
[--plan poses.csv]
[--figures/--no-figures]`

Two passes: pass 1 segments every pose by the closest large plane (or by prediction
from `--sensor-in-base` if given) and registers; pass 2 segments every pose with the
prediction from the pass-1 transform and registers again. Both models are solved when
`both`. Outputs: `registration.json` (parameters, per-pass transform matrices, scale,
status, spreads, RMS/max residuals, per-pose residuals and segmentation statistics,
rejected poses), `segmentation.npz` (the mask of every pose of every model and pass,
keyed `<model>_pass<n>__<pose_id>`, since masks differ per model and pass), and the
figures of 8.2 in `figures/<model>/` when enabled. `--truth truth.json` (a simulated
session) adds the errors against the true transform. Exit 1 when any requested
model's final status is not Success.

### 8.2 `analysis/residual_maps.py`

Figures (matplotlib, PNG, 150 dpi, light surface, legend whenever two or more
series, text in ink colors not series colors, a single-hue sequential map for
magnitudes, a two-hue diverging map with a gray midpoint for signed residuals, 2 px
lines, markers at least 8 px):

1. `pose_residuals.png`: per-pose normal residual (deg) and offset residual (mm)
   against pose index, two panels, used and rejected poses distinguished.
2. `normal_residual_field.png`: a **vector map** over the image: at each pose's board
   center (u, v), an arrow for the normal residual vector `R n_k - m_k` projected onto
   the sensor's image axes, scaled so that the longest arrow is readable and the scale
   bar states degrees; marker color by standoff (sequential), one panel per standoff
   when standoffs are discrete, else one panel.
3. `offset_residual_field.png`: at each board center, a marker colored by the signed
   offset residual (diverging), over the image frame; one panel per standoff.
4. `pixel_residuals_<pose_id>.png` for a selectable subset (`--pixel-maps N`, default
   the 6 poses with the largest |offset residual| plus 2 random): the per-pixel
   signed distance of every mask pixel to the plane predicted from the robot pose
   through the final transform (diverging, symmetric limits), with a subsampled
   quiver of the in-image gradient of that residual (the local tilt error), and the
   mask outline.
5. `comparison.png` when `analysis/compare.py` is run on two sessions (A and B):
   overlaid histograms of the two residual sets and a table of RMS values in the
   title.

### 8.3 `analysis/compare.py`

`--registration-a registration.json --registration-b registration.json --out DIR`:
compares two sessions of the same plan (sub-procedures A and B): per-pose residual
differences, RMS of each, the relative transform between the two solutions (rotation
angle, translation distance), writes `comparison.json` (with a `b_smaller_rms` flag)
and `comparison.png`. Exit 0 whatever the outcome; the comparison is a report, not a
test.

### 8.4 `analysis/report.py`

`--registration registration.json [--comparison comparison.json] --out report.md`:
a Markdown report with the transform, the acceptance verdict, the residual table,
the spreads against their thresholds, the segmentation summary (methods, pixel
counts, planes rejected), and the list of figures. No prose generation beyond fixed
sentences with the numbers filled in.

### 8.5 `analysis/simulate.py`

`python3 -m planereg.analysis.simulate --out DIR [--plan poses.csv | default plan]
[--frames 3] [--sensor-to-base JSON | random seeded] [--clutter] [--flyaways]
[--backlash-deg X --backlash-mm Y] [--procedure A|B] [--seed]`

The default plan (no `--plan`) is a small self-contained grid of 24 poses at
standoffs 450, 600 and 750 mm (at 850 mm and 25 degrees a board on a 320 x 240 test
camera falls below the segmentation's minimum pixel count). `--scale C` makes the
true map X = [C R, s; 0 1], the rendered XYZ being divided by C, so a similarity
registration recovers C. The simulated table ends under the board (it exists only at
depths at or beyond the board center) so that the closest-large-plane search of pass
1 is exercised but not defeated; a real session must keep the foreground clear
(procedure, Section 10).

Writes a synthetic session in the layout of a real one: `captures/<pose_id>_Index<nn>.mc`
(XYZ float32 via `write_matcloud`, header fx fy cx cy h v cameraName version and
`robotPose` = the **reported** board-to-base pose), `manifest.json`, `pose_log.csv` in
the technician's format (with `j1..j6` columns filled from a fake kinematic model only
if the agent finds that cheap; otherwise omitted), `truth.json` (true X, true poses,
generator parameters). Rendering: `render_board_frame` with a zero error field by
default (`--error-field default` switches on sphcal's bowl and slope field);
`--clutter` adds a wall plane 300 mm behind the board filling the field and a table
plane below; `--flyaways` displaces a fraction (0.3) of the pixels within 2 px of
the board silhouette along their rays by a random factor in [0.9, 1.1]. Backlash
emulation: the true pose differs from the reported pose by a rotation of
`backlash_deg` about a fixed base-frame axis and a translation of `backlash_mm` along
a fixed base direction, with a random sign per pose in procedure A and a constant
sign in procedure B. The true pose is the reported pose rotated about the fixed
base-frame axis through the board origin and shifted along a fixed base direction.
This is a stand-in for testing the comparison, not a model of any robot. Note what it
implies: a constant-sign backlash is absorbed into the solved transform (procedure B
registers with small residuals but is offset from the true sensor placement), while
a random-sign backlash shows as residual scatter (procedure A). The comparison of
the two sessions therefore measures the backlash's size, not which session is
"right"; the engineer decides which to trust from the residuals and the joint-sign
report.

## 9. Tests (`python/tests/`)

- `test_core.py`: registration recovers a random X exactly from exact planes (rigid
  and similarity, 1e-9); segmentation chooses the board over a larger wall behind it
  and over a small closer patch; fly-aways on the silhouette do not move the fitted
  plane by more than 0.02 deg / 0.02 mm relative to the clean render; the predicted
  region finds the board when the rough transform is off by 2 deg and 20 mm.
- `test_capture_tools.py`: `plan_poses` on a synthetic camera produces about 200
  poses by default, every board inside the image, the approach columns present and
  the approach pose at the stated retreat and rotation from the target; its
  `poses.csv` is accepted by `sphcal.cli.make_manifest`; `bootstrap` recovers the
  synthetic X within 0.1 deg / 1 mm from 6 poses; `check_captures` passes a clean
  session and flags a pose whose logged position was shifted by 10 mm.
- `test_analysis_tools.py`: `register` on a synthetic session (clutter and fly-aways
  on) recovers X within 0.05 deg / 0.5 mm, both passes; the similarity model recovers
  an injected scale of 1.02 within 1e-3; `compare` on two synthetic sessions with
  backlash emulation reports a smaller residual RMS for procedure B; `report` writes
  a file containing the transform.
- Every test uses a small camera (160 x 120 or 320 x 240) so the suite runs in under
  a minute.

## 10. Parameters that the procedure document quotes

Plan defaults (7.1), approach retreat 40 mm and rotation 3 deg, 5 frames per pose,
settle time 1.5 s, check thresholds (7.3), registration thresholds
(`RegistrationParameters`), segmentation thresholds (`SegmentationParameters`). If an
implementation changes a default, the procedure document must be updated with it.
