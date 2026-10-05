# Registration captures: step-by-step procedure

Audience: the robot technician who will set up the board, program the robot, and
record the captures. No knowledge of the registration math is needed. Where a step
says "run", a computer with Python and this repository is needed (appendix B says
how to set it up); the engineer can run those steps for you if you send the files.

What you are producing: two folders of sensor capture files (`.mc`), one group of
files per robot pose, each folder with a small table (the manifest) that says, for
every file, exactly where the robot had put the board. The two folders come from the
same list of poses, driven in two different ways (section 6): one ignoring the
robot's gear backlash and one trying to make its effect the same at every pose. The
registration software works out from each folder where the sensor is relative to
the robot, and the engineer compares the two. If the table is wrong, the
registration is wrong, so most of this procedure is about getting the table right.

This procedure follows the stage-1 calibration capture procedure of the depth
calibration project in its layout and its tools; the board, its adapter and its tool
frame are the same, except that the board may be thinner (6 mm instead of 15 mm;
section 1 says why). A stage-1 board can be used as it is. The spheres and the
three-ball nest are not needed here.

Terminology: in this project "registration" means finding the transform between the
sensor and the robot (the extrinsic parameters); "calibration" means finding the
sensor's own internal parameters. This procedure is about registration only.

Figure 1 (`figures/fig_approach.png`, section 6) shows the two ways of driving to a
pose; figure 2 (section 5) shows an example pose plan.

---

## 1. Equipment

| Item | Requirement | Notes |
|---|---|---|
| Sensor | The 3D sensor to be registered, rigidly mounted; it must not move during the whole session | Mount on a stiff bracket, not a tripod; mark its position so a bump is noticed |
| Robot | Six-axis industrial robot, absolute positioning accuracy 0.1 mm or better over the working volume, with a 50 mm or larger ISO flange | A robot calibrated by its maker ("absolute accuracy" option) is needed; repeatability alone is not enough |
| Board | Flat plate 200 mm x 150 mm, at least 6 mm thick, front face matte and light gray, flat to 0.05 mm after finishing | Ground aluminum tooling plate or float glass, matte-painted; ask the supplier for a flatness report (stage-1 procedure, appendix A, lists suppliers). Thickness is not a stiffness matter (a 6 mm plate sags about 0.002 mm under its own weight); it only has to survive the finish and the mounting flat. Do not bead-blast a plate thinner than about 10 mm: peening one face bows it. Paint over the ground or glass surface instead, and check the flatness after painting and mounting, not before |
| Board adapter | Plate that bolts to the flange with the dowel pins and holds the board with its front face perpendicular to the flange axis and its center on the flange axis | Three-point mounting (two dowels and a clamp) so the board goes back in the same place |
| Dial indicator with magnetic base | 0.01 mm resolution | Board runout check |
| Depth gauge or calipers | 0.02 mm resolution | Board offset D |
| Capture computer | Runs the sensor's capture software, writes `.mc` files with the sensor's own name in the file name | At least 10 GB free per 1,000 frames |

Appendix A lists the software tools used in sections 4 to 8 and their options;
appendix B says how to install them.

## 2. Before anything else

Switch the sensor on and leave it running for at least 30 minutes before the first
capture, and leave it running for the whole session. Note the time it was switched on.

Fix the sensor's exposure and gain to the values that will be used in production.
Write them down. Do not use automatic exposure.

Switch off or block any sunlight or lamps that fall on the board. Room light is fine
if it is constant.

Confirm with the engineer what the robot base frame is (which frame the robot's
position readout is in). Every pose in this procedure is recorded in that frame. Do
not change the active base frame during the session.

Leave the controller's backlash compensation (if it has such a setting) exactly as it
is in production, and write down what it is set to. Do not change it between the two
sub-procedures of section 6.

Record in a text file (`session_notes.txt`): date, sensor serial number, exposure
and gain, robot model and controller software version, the active base frame name,
the backlash compensation setting, the board certificate (flatness), room
temperature, and anything unusual.

## 3. Mounting the board and its tool frame

The board's "tool frame" is a coordinate frame at the center of the board's front
face, with its z axis pointing straight out of the face (toward the sensor when the
board faces it), x along the long edge, y along the short edge. The robot must report
the board's pose in this frame. This is the same frame as TOOL_BOARD of the stage-1
procedure; if it is already defined on this robot with this board and adapter, check
it (steps 2 and 3) and reuse it.

1. Mount the board adapter and the board on the flange with the dowel pins.
2. Runout check: fix the dial indicator to the table with its tip on the board's
   front face about 20 mm from an edge. Slowly rotate the flange about its own axis
   (robot joint 6) through 360 degrees. The reading must stay within 0.05 mm. If it
   does not, the board face is not perpendicular to the flange axis: shim the adapter
   and repeat.
3. Measure the distance from the flange face to the board's front face with a depth
   gauge or calipers at four places around the board; they should agree within
   0.05 mm. Record the average as D.
4. Measure the board's width and height with calipers and record them. The
   half-sizes go in the pose log (100 and 75 mm for the recommended board).
5. Define the tool frame in the robot: position (0, 0, D) from the flange,
   orientation: z along the flange axis pointing out of the board, x along the board's
   long edge. An error of a few degrees in x is harmless (the board is symmetric); an
   error in z is not.
6. Save as TOOL_BOARD and write the numbers in the session notes.

If the controller cannot define a tool frame and can only report the flange, log the
flange pose instead and tell the engineer the value of D; the software has a setting
for it (`--target-offset-mm`). Do not mix the two in one session.

## 4. Finding where the sensor is (rough)

The software works out the exact position of the sensor from the captures. It only
needs a rough idea first, so that the planned poses land in the sensor's field of
view and so that the software knows where to look for the board in each capture.
This step takes six captures of the board at hand-chosen poses.

1. With TOOL_BOARD active, jog the board to a point roughly in front of the sensor,
   about 600 mm away, near the middle of the picture, facing the sensor squarely.
   Check on the capture computer's live view that the whole board is visible with
   margin and that nothing else flat and large is closer to the sensor than the
   board (a tabletop between the sensor and the board, for instance, must be out of
   the picture or farther away than the board).
2. Record a capture (one frame is enough), read the robot's reported tool pose
   (position and orientation, in the base frame), and append it to the pose log
   (section 7) with the pose id `boot01`. Name the file `boot01_Index00.mc`.
3. Tilt the board about 20 degrees so that its top edge comes toward the sensor;
   capture `boot02` and log the pose. Then about 20 degrees the other way (`boot03`);
   then 20 degrees with the left edge toward the sensor (`boot04`); then the right
   edge (`boot05`). Finally move the board about 250 mm farther from the sensor,
   facing it squarely (`boot06`). Six poses, tilted in four directions and at two
   distances.
4. Build the manifest and run the bootstrap (the engineer can do this):

```
python3 -m sphcal.cli.make_manifest --pose-log boot_pose_log.csv --captures boot/ --format csv --out boot/manifest.csv
python3 -m planereg.capture.bootstrap --manifest boot/manifest.csv --out boot.json
```

The bootstrap prints one line per pose with the board it found (how many pixels,
how flat), and after solving, each pose's disagreement with the solution as an angle
and a distance. They should be a fraction of a degree and a millimeter or two at
most; a disagreement of several degrees or tens of millimeters means a pose was
copied wrongly, the tool frame's z axis is wrong (section 3, step 5), or the board
was not the closest flat surface in that capture. It also prints the "normal spread"
of the six poses, which says whether the tilts were varied enough: it must be at
least 0.05 (the tilts above give about 0.15). `boot.json` is the rough sensor
position; the next steps read it.

## 5. The pose plan

Run the plan tool with the rough sensor position, one of the bootstrap captures (for
the sensor's field of view) and the measured board half-sizes:

```
python3 -m planereg.capture.plan_poses --sensor-in-base boot.json --camera boot/boot01_Index00.mc \
    --board-half-size-mm 100 75 --out plan/
```

(Use your board's half-sizes in place of 100 75.) It writes `plan/poses.csv`,
`plan/plan_summary.txt` and a picture `plan/plan.png` (figure 2 shows the picture for
the default settings). The plan with the default settings is:

The board at three standoffs from the sensor (550, 750 and 950 mm), at a 3 x 3 grid
of lateral positions covering 50 percent of the field at each standoff, facing the
sensor and tilted by 0, 15 and 30 degrees, the tilted poses in four directions (top,
right, bottom and left edge toward the sensor): 9 orientations at each of 27
positions. Tilted boards that would leave the field of view are dropped
automatically. About 20 percent of the poses are marked holdout in the table;
capture them like all the others, the software keeps them for checking.

{{PLAN_PARAGRAPH}}

The pose budget. This is the first run: a few hundred poses are enough to find out
whether the robot's pose errors or the sensor's plane errors dominate. The plan
summary gives the registration error to expect from the plan if the sensor's planes
are good to 0.2 degrees and 0.2 mm; if the engineer's analysis of the first run
calls for a larger set, a second run of 1,000 or more poses is planned with
`--target-pose-count`, not before.

![plan](figures/fig_plan_example.png)

Figure 2. The planned board centers for the default settings, in the sensor's frame:
side view (left) and front view (right), with the field of view drawn. One color per
standoff; held-out poses have an outline.

Every row of `poses.csv` gives: `pose_id`, `kind` (always board), the board
half-sizes, `holdout`, the position of the board tool frame in the base frame
(`base_x_mm`, `base_y_mm`, `base_z_mm`) and its orientation three ways (rotation
matrix `r00..r22`, quaternion, rotation vector; use whichever your robot program
accepts), the standoff, tilt and azimuth the pose was planned with, and the
**approach pose** for sub-procedure B: its position (`approach_x_mm`, `approach_y_mm`,
`approach_z_mm`) and rotation matrix (`approach_r00..approach_r22`). Hand the file to
whoever writes the robot program, or import it directly if the controller can read
CSV.

## 6. The two sub-procedures

Every pose is captured twice, in two separate runs over the whole plan, with the
sensor and the board untouched between them. Figure 1 shows the difference.

![approach](figures/fig_approach.png)

Figure 1. Left: sub-procedure A drives straight onto each target pose. Right:
sub-procedure B drives to the approach pose first, then makes the same final move
onto the target at every pose. (The approach rotation is exaggerated in the drawing.)

**Sub-procedure A, "ignoring backlash".** Robot program outline, for each row of
`poses.csv`:

1. Move to the target pose with one joint move, from wherever the robot is.
2. Wait 1.5 seconds for vibration to settle.
3. Trigger the capture of 5 frames; file names `<pose_id>_Index00.mc` to
   `<pose_id>_Index04.mc`, in the folder `captures_A/`.
4. Read the robot's actual reported tool pose (not the commanded one, the reported
   one) and, if the controller gives them, the six joint angles; append them to the
   pose log `pose_log_A.csv` (section 7).
5. Move on.

**Sub-procedure B, "minimizing backlash".** The idea: gear backlash means the flange
can sit on either side of a small dead band after a move, depending on which way each
joint last moved. Approaching every target pose with the same final move makes the
last motion of each joint the same at every pose, as far as the geometry allows, so
the dead band is taken up the same way each time and its effect is constant instead
of random. The approach pose in the plan is the target pose moved back 40 mm along
the board's own normal (away from the sensor) and turned 3 degrees about the board's
long edge; the final move is therefore the same 40 mm advance and 3 degree turn at
every pose. Robot program outline, for each row:

1. Move to the approach pose with a joint move, from wherever the robot is.
2. Wait 0.5 seconds.
3. Make a linear move onto the target pose, at the same speed for every pose, without
   stopping or reversing on the way. If the controller offers a "fine" or "exact
   stop" positioning mode for the end of a move, use it here and in sub-procedure A
   alike.
4. Wait 1.5 seconds for vibration to settle.
5. Trigger the capture of 5 frames; file names as above, in the folder `captures_B/`.
6. Read the reported tool pose and, if available, the joint angles at the target,
   and append them to `pose_log_B.csv`; if the controller can also report the joint
   angles at the approach pose, log those in the `approach_j1..approach_j6` columns.
   The check tool uses them to confirm that each joint's final motion had the same
   direction at every pose.
7. Move on.

Use the same speed and acceleration settings in both sub-procedures. Do all of A,
then all of B (or the other way round); do not interleave, and do not move the
sensor or remount the board between them. At about 8 seconds per pose in A and 12 in
B, the robot time for the default plan of about 240 poses is about 80 minutes in
all; plan on about three hours including the setup and the checks.

## 7. Recording the poses: the pose log and the manifest

The capture software writes the sensor data. The pose must be recorded separately,
in one of two ways. Both are supported; use the first if the capture software can do it.

Option A, pose in the file header. If the capture software can be given the robot's
pose at capture time, it writes it into the file's header under the key `robotPose`
as a 4 x 4 matrix (16 numbers, row by row: rotation in the top-left 3 x 3, position
in the right column, last row 0 0 0 1), tool frame to base frame. Then the pose log
is only needed for the joint angles.

Option B, pose log plus manifest (preferred when option A is not available, and
recommended anyway as a backup). The robot program appends one line per pose to a
CSV file, the pose log, with these columns:

```
pose_id, kind, radius_mm, half_width_mm, half_height_mm, x_mm, y_mm, z_mm, rotation_type, r1, r2, r3, r4, j1, j2, j3, j4, j5, j6, approach_j1, ..., approach_j6
```

- `pose_id`: exactly the id from `poses.csv`, which is also the start of the file names.
- `kind`: `board`. `radius_mm`: empty.
- `half_width_mm`, `half_height_mm`: half the measured board size.
- `x_mm`, `y_mm`, `z_mm`: the reported tool position in the base frame.
- `rotation_type` and `r1..r4`: the reported tool orientation, in whatever form the
  controller gives, named by one of: `quaternion_wxyz`, `quaternion_xyzw`,
  `euler_zyx_deg` (KUKA A, B, C), `fixed_xyz_deg` (FANUC W, P, R), `euler_xyz_deg`,
  `rotvec_deg`. Fill unused r columns with nothing.
- `j1..j6` and `approach_j1..approach_j6`: the joint angles in degrees, if the
  controller reports them; otherwise leave the columns out entirely.

Example line:

```
b_z0650_t24_a090_05,board,,100.0,75.0,850.11,-12.70,470.55,euler_zyx_deg,-91.3,19.8,0.4,,12.1,-35.6,88.2,3.4,41.0,-8.7,11.9,-36.0,88.9,3.1,40.5,-8.4
```

After each sub-procedure, the manifest is built from its pose log and its capture folder:

```
python3 -m sphcal.cli.make_manifest --pose-log pose_log_A.csv --captures captures_A/ --format csv --out captures_A/manifest.csv
python3 -m sphcal.cli.make_manifest --pose-log pose_log_B.csv --captures captures_B/ --format csv --out captures_B/manifest.csv
```

The tool matches every file to its pose id, converts the orientation to the standard
matrix form, and complains, by name, about any pose without files, any file without
a pose, and any missing size. A `poses.csv` from the plan tool is also accepted
directly as the pose log if your robot program cannot write one; then the commanded
poses stand in for the reported ones, which is acceptable only for a robot with
absolute accuracy better than 0.1 mm, and it defeats the purpose of sub-procedure B.
Extra columns such as the joint angles are carried along unchanged.

## 8. Checking the captures before the analysis

Run the quick check on each folder as soon as its captures are complete, while the
robot and the board are still set up:

```
python3 -m planereg.capture.check_captures --manifest captures_A/manifest.csv --sensor-in-base boot.json --pose-log pose_log_A.csv --out captures_A/check.json
python3 -m planereg.capture.check_captures --manifest captures_B/manifest.csv --sensor-in-base boot.json --pose-log pose_log_B.csv --out captures_B/check.json
```

It prints one line per pose and a verdict. Things it flags, and what they mean:

- Low valid fraction (below 50 percent of frames): the board was not read; check
  exposure, or the board was outside the field.
- Board touching the image border: the board is partly out of view; the pose is
  unusable, re-plan it slightly inward.
- Segmentation failed: the software could not find a flat surface of the board's
  size facing the sensor where the pose says it should be; the pose was copied
  wrongly, or something flat is closer to the sensor than the board (a cable, a
  hand, the table), or the board is outside the field.
- Plane residual above 1 mm: the board surface read by the sensor is not flat to
  the sensor; a shiny spot or a dirty board, or an exposure problem.
- Normal residual above 1 degree or offset residual above 1 mm against the solved
  registration: either the robot pose was copied wrongly for that pose, or, if it
  affects every pose with one tilt direction, the board tool frame's orientation is
  wrong (section 3, step 5), or the rotation type in the pose log is misnamed; or,
  if it grows steadily across the volume, the robot's base frame or its absolute
  accuracy is off.
- Normal spread below 0.05: the tilts were not varied enough (should not happen with
  the plan's defaults).
- Joint sign consistency (sub-procedure B, when joint angles were logged): for each
  joint, the fraction of poses whose final motion had the majority direction. A joint
  far below 100 percent means the approach move did not standardize that joint's
  backlash over part of the workspace; tell the engineer, it affects how the two
  folders are compared.

Re-capture flagged poses after fixing the cause; do not delete the lines from the
log, add corrected lines with a new pose id (for example `b_z0650_t24_a090_05r`). The
check tool's exit code is 0 when nothing is flagged, 1 when something is, and 2 when
it cannot read the manifest.

## 9. Deliverables checklist

- `captures_A/` and `captures_B/` with all `.mc` files, named `<pose_id>_Index<nn>.mc`
- `pose_log_A.csv` and `pose_log_B.csv` (option B), or confirmation that headers carry
  `robotPose` (option A) plus the joint-angle logs
- `captures_A/manifest.csv` and `captures_B/manifest.csv` produced by `make_manifest`
  without errors
- `captures_A/check.json` and `captures_B/check.json` with no flags, or a note
  explaining each remaining flag
- `plan/poses.csv`, `plan/plan_summary.txt`, `plan/plan.png` as used
- `boot.json`, `boot_pose_log.csv` and the bootstrap captures in `boot/`
- `session_notes.txt` with: sensor serial, warm-up time, exposure and gain, base
  frame name, backlash compensation setting, board D and runout reading, board
  dimensions and certificate, TOOL_BOARD values, temperature, robot speed and
  acceleration settings, the order the sub-procedures were run in and their times
- Photos of the setup: sensor mount, the board on the flange, the workspace behind
  the board as the sensor sees it

## 10. Things that spoil a session

- Moving or bumping the sensor. If it happens, everything after it is a new session,
  and both sub-procedures must be repeated.
- Changing exposure, gain, the base frame, the speed settings, or the backlash
  compensation mid-session or between the sub-procedures.
- Using the commanded pose instead of the reported pose in the log.
- A loose adapter: check the dowels are seated and the runout (section 3, step 2)
  before each sub-procedure.
- Fingerprints or gloss on the board: wipe with isopropyl alcohol; a shiny spot
  returns a bright highlight and a bad read.
- Something flat and large closer to the sensor than the board, in the picture. The
  software looks for the closest large flat surface when it has no better
  information; keep the foreground clear.
- In sub-procedure B, stopping, reversing, or slowing on the final move, or
  approaching some poses without the approach pose. Every pose must get the same
  final move.
- Mixing up the two pose logs, or capturing B into the A folder.

## Appendix A. Software reference

The tools live in this repository under `python/planereg/` (the `capture` package)
and use the sibling repository `depth_calibration_from_spherical_target` (the `sphcal`
package) for reading the capture files and building the manifest. Every tool prints
its help with `--help`.

{{PLAN_HELP}}

{{BOOTSTRAP_HELP}}

{{CHECK_HELP}}

Output of `python3 -m sphcal.cli.make_manifest --help` is in the stage-1 procedure,
appendix B.

## Appendix B. Installing and running the software, step by step

These steps follow appendix C of the stage-1 procedure; the only difference is that
two repositories are installed, `sphcal` first and `planereg` second. Allow about 20
minutes. Nothing here needs administrator rights.

B.1 Install Python 3.10 or newer (stage-1 procedure, appendix C.1).

B.2 Get the code: unzip the two archives the engineer sent, or clone the two
repositories, side by side, somewhere without spaces in the path, for example
`C:\cal\depth_calibration` and `C:\cal\flexible_plane_fit` on Windows or
`~/cal/depth_calibration` and `~/cal/flexible_plane_fit` on Linux.

B.3 Open a terminal in `flexible_plane_fit/python` (stage-1 procedure, appendix
C.3). Check with `dir` (Windows) or `ls` (Linux) that `pyproject.toml` is listed.

B.4 Make a private Python environment and install both packages into it:

```
python3 -m venv .venv
.venv\Scripts\activate        (Windows)
source .venv/bin/activate     (Linux)
python3 -m pip install -e ../../depth_calibration   (the sphcal repository, by its path)
python3 -m pip install -e ".[figures,test]"
```

The prompt now starts with `(.venv)`; activate it again in every new terminal.

B.5 Run the self-test, still in `flexible_plane_fit/python`:

```
python3 -m pytest -q tests/test_capture_tools.py
```

It must end with a line like `N passed in 20s`. Any line containing `FAILED` or
`ERROR` means the installation is not right; copy the whole output into a text file
and send it to the engineer. Do not start capturing until this passes.

B.6 Run the tools as `python3 -m planereg.capture.<tool>` (and
`python3 -m sphcal.cli.make_manifest`) followed by their options, as in sections 4
to 8. With the environment active this works from any folder, so keep a working
folder for the session with `boot/`, `plan/`, `captures_A/`, `captures_B/` and the
pose logs in it, open the terminal there, and give file names relative to it. A
message starting with `ERROR:` means the tool stopped and wrote nothing; it says what
is wrong and which pose or file it concerns. A message starting with `WARNING:` means
the tool finished but something should be looked at.

B.7 If something goes wrong: the stage-1 procedure's appendix C.7 applies unchanged
(`No module named sphcal` or `No module named planereg` means the environment is not
active or the install step was skipped).
