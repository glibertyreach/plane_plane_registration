# Prior art: registration of a 3D sensor to a robot from plane-plane correspondences

Scope of the question. The method of "Registration to Plane: A Cookbook" registers a
fixed 3D sensor into the robot frame by observing a plain planar plate held on the
flange at several poses, matching each measured plane to the plate plane transformed
by the robot pose, and solving the rotation from the plane normals and the
translation from the plane offsets. The claimed novelty is that this needs only a
planar target and only 3D (plane) measurements, with no 2D image acquisitions.

Terminology. This note uses the author's convention: "registration" is the
determination of the sensor-to-external-frame transform (extrinsic parameters) and
"calibration" the determination of sensor intrinsics. The literature almost
universally calls the former "hand-eye calibration" or "extrinsic calibration", so
those words appear below only inside titles and quotations.

How the search was done. Web search (academic and patent full-text indexes via
Google Patents, arXiv, publisher sites, Crossref) in October 2026, with the papers
that mattered most read in full text rather than from abstracts. The USPTO Open
Data Portal was not used: the API key lives on the author's local drive, which this
session cannot reach. Every citation below was checked against Crossref or the
publisher page unless marked otherwise. Assessments of what "anticipates" what are
a technical reading, not a legal opinion.

---

## 1. Summary

The broad idea is not new. Registering a 3D range sensor to a robot by fitting
planes to range data at several robot poses, with no 2D features, is in the
academic literature from 2008 and in a granted US patent with a 2017 priority date.
The specific mathematics, rotation by aligning plane normals and translation by a
linear solve on plane offsets, is the standard closed form for rigid motion from
plane correspondences and dates to the 1980s.

What I did not find is a publication or patent with exactly the spec's
configuration: sensor fixed, a single plain plate on the flange whose plane is known
in the flange frame (a thickness), full 3D planes measured, and a closed-form
non-iterative solve. Each of those elements appears in the prior art, in different
combinations. In my (non-legal) judgment the combination would be hard to defend as
inventive against the ABB patent and the 2008 papers, because the fixed-sensor and
moving-sensor arrangements are the same equations with the robot pose inverted, and
the single moved plate supplies the same diversity of plane orientations that a
multi-faceted fixed article supplies geometrically.

The closest references, in order of relevance:

| # | Reference | What it has | What it lacks relative to the spec |
|---|---|---|---|
| 1 | ABB, US 10,661,442 B2 (priority 3 Feb 2017, granted 26 May 2020), "Calibration article for a 3D vision robotic system" | 3D camera on the robot; planar faces found by RANSAC in the point cloud; plane equations `[n, d]` matched against stored plane equations; rotation by aligning normals, translation by linear least squares; no 2D features | Camera on the robot, not fixed; a multi-faceted article with known face planes, not a single plate; plane equations known in the article frame, not the flange frame |
| 2 | Fuchs and Hirzinger, CVPR 2008, "Extrinsic and depth calibration of ToF-cameras" | ToF camera on a robot, one calibration plane (a wall) fixed in the workspace, observed at many robot poses; sensor-to-TCP transform estimated from point-to-plane distances only, no 2D features | Sensor on the robot; plane pose unknown and estimated jointly; nonlinear optimization, not closed form; the wall carries a checkerboard, but only to expose depth errors on dark and bright areas, not for 2D features of the extrinsic estimate |
| 3 | Kaiser, Tauro, Wörn, IJISTA 5(3/4), 2008, "Extrinsic calibration of a robot mounted 3D imaging sensor" | Hand-eye transform of a robot-mounted 3D sensor using the floor of the robot cell as the (planar) calibration rig | Sensor on the robot; a single fixed plane; only the abstract was accessible to me, so the solution method is unverified |
| 4 | Bagge Carlson, Johansson, Robertsson, IROS 2015, "Six DOF eye-to-hand calibration from 2D measurements using planar constraints", DOI 10.1109/IROS.2015.7353884 | Wrist-mounted laser scanner, measurements from at least three non-parallel fixed planes of unknown equation; plane normals by PCA, offsets encoded in the normal; rotation projected onto SO(3) by SVD; no 2D imaging | Sensor on the robot, planes fixed and unknown; a 2D (line) sensor, so each observation is a line in the plane rather than the plane; linear but iterative |
| 5 | Wang, Jing, Liao, Ma, Zheng (PlaneHEC), arXiv 2507.19851, 26 July 2025, "PlaneHEC: Efficient hand-eye calibration for multi-view robotic arm via any point cloud plane detection" | Depth camera on the robot observing any plane (wall, table) at several poses; planes transformed as homogeneous vectors (`Y = M X A`), closed-form rotation from normals by SVD, translation by a linear solve, then iterative refinement | Postdates the spec (first release 15 April 2025) and so is not prior art, but it is an independent statement of the same mathematics in the dual (eye-in-hand) arrangement; its own related-work section names refs. 3 and 4 as the plane-constraint precedents |
| 6 | Aisin Seiki, US 7,822,571 B2 (priority 28 Feb 2008), "Calibration device and calibration method for range image sensor" | Range image sensor; two or three non-parallel plane surfaces on a fixed target; the sensor rotation from the measured plane normals against the known target posture; translation from plane intersections; no 2D features | Vehicle-mounted, no robot; the target is fixed, so the orientation diversity comes from the target's geometry, not from motion |
| 7 | Wan and Song, Frontiers in Robotics and AI 7:65, 2020, "Flange-based hand-eye calibration using a 3D camera with high resolution, accuracy, and frame rate", DOI 10.3389/frobt.2020.00065 | Fixed 3D camera (eye-to-hand); the robot presents its bare flange at several poses; the flange plane is found by RANSAC plane fitting and the flange circle by RANSAC; Umeyama closed form; no 2D features, no target at all | Uses the flange circle center as a point correspondence for the transform; the plane is used to locate the circle, not as the correspondence itself |

Foundational mathematics (not robot-specific) that the spec reproduces:

- Faugeras and Hebert, IJRR 5(3):27-52, 1986, DOI 10.1177/027836498600500302, "The
  representation, recognition, and locating of 3-D objects": rigid motion from plane primitives `(n, d)`, rotation
  from the normals (quaternion least squares), translation from the offsets.
- Khoshelham, ISPRS J. Photogramm. Remote Sens. 114, 2016, DOI
  10.1016/j.isprsjprs.2016.01.010, "Closed-form solutions for estimating a rigid
  motion from plane correspondences extracted from point clouds", and Förstner and
  Khoshelham, ICCV Workshops 2017, DOI 10.1109/ICCVW.2017.253, "Efficient and
  accurate registration of point clouds with plane to plane correspondences": the
  direct solutions, including the role of plane normalization, which is exactly the
  normalization issue flagged in `REVIEW.md` Section 4.2.
- MERL, US 9,183,631 B2, "Method for registering points and planes of 3D data in
  multiple coordinate systems": plane-plane and point-plane correspondences for
  rigid registration of 3D data (not read in full; listed from the search index).
- Zhang and Pless, IROS 2004, "Extrinsic calibration of a camera and laser range
  finder (improves camera calibration)" and Unnikrishnan and Hebert, CMU-RI-TR-05-09,
  2005, "Fast extrinsic calibration of a laser rangefinder to a camera": the
  plane-constraint formulation for sensor-to-sensor registration, closed-form
  rotation from normals then translation, refined nonlinearly. Ref. 4 cites Zhang
  and Pless as its starting point.

---

## 2. Why the eye-in-hand references read on the eye-to-hand method

Let `X` be sensor-to-world and `T_k` flange-to-world. The spec's correspondence is

    X^{-t} p_k = T_k^{-t} o                      (sensor fixed, plate on flange)

For a sensor on the flange with transform `Y` (sensor-to-flange) and a fixed plane
`q` in the world, the correspondence is

    (T_k Y)^{-t} p_k = q      ⇔      Y^{-t} p_k = T_k^{t} q

Both say "an unknown rigid transform maps the measured plane to a plane that the
robot pose makes known", and both split into `R n_k = m_k` for the rotation and a
linear equation in the translation. The only structural difference is that in the
fixed-plane case `q` is usually unknown (three extra unknowns, as in refs. 2 and 4),
while in the spec the plate plane `o` is known in the flange frame. Knowing `o`
makes the spec's problem strictly easier and its solution closed-form, but it does
not change the kind of measurement, the kind of target, or the kind of
computation. That is why I rate the 2008 papers and the ABB patent as anticipating
the method's substance.

---

## 3. Notes on each reference

### 3.1 ABB, US 10,661,442 B2 (Wang, Boca, Zhang)

Read from the Google Patents record. Claim 1 recites storing 3D shape data of a
calibration article with at least four inward-angled planar side faces, capturing
3D images with a robot-carried 3D camera, identifying the planes, and computing the
camera-to-article transform. The description stores "the plane equation parameter
`[nx, ny, nz, d]` for each surface" in the article frame, finds dominant planes by
RANSAC, aligns the measured normals with the stored ones for the rotation, and
solves the translation by linear least squares from the matched plane equations.
No 2D features. This is plane-plane registration of a 3D camera using only planar
faces with known plane equations in a second frame, which is the spec's method with
the roles of the known and unknown frames rearranged. Whether the claims, which are
tied to the article's geometry, would block a single-plate method is a legal
question I cannot answer; as a disclosure it anticipates the concept.

### 3.2 Fuchs and Hirzinger, CVPR 2008 (DOI 10.1109/CVPR.2008.4587828)

Read in full. "The ToF-camera is mounted on the robot and moved to N different
poses. The calibration plane is defined by its normal `n_c` and its distance `d_c`
to the origin of world coordinates. The poses are given by the robot control." The
hand-eye transform and the plane pose are unknown and estimated by nonlinear
optimization of the squared distances between measured points and the plane, jointly
with a depth-correction model (intrinsic calibration). A checkerboard is printed on
the plane only so that both dark and bright regions are measured; the extrinsic
estimate uses no 2D features. This is the earliest full-text reference I found for
"register a 3D sensor to a robot with a plane and depth data only".

### 3.3 Kaiser, Tauro, Wörn, IJISTA 2008 (DOI 10.1504/IJISTA.2008.021300)

Abstract only (the publisher page refused access): "a technique for estimating the
hand-eye transformation for a robot mounted 3D imaging sensor ... uses the ground
floor of the robot cell as a calibration rig". PlaneHEC characterizes it as
requiring the plane to be measured manually. A single fixed plane with unknown pose
cannot determine all six degrees of freedom from plane constraints alone (the spec's
own rank analysis in `REVIEW.md` Section 4.6 applies in the dual form), so this
reference must rely on additional knowledge of the floor plane; I could not confirm
how.

### 3.4 Bagge Carlson, Johansson, Robertsson, IROS 2015

Read in full. Despite the title, the sensor is wrist-mounted ("the eye-to-hand
calibration problem between a wrist-mounted laser scanner and the tool flange"); the
planes are fixed in the cell, at least three, non-parallel, with unknown equations,
and the method is "linear, iterative": plane equations are estimated by PCA from
the points mapped with the current transform estimate, then the transform is
re-solved from the requirement that all measured points satisfy their plane
equations, with the rotation projected to the closest orthonormal matrix by SVD.
At least nine points from at least three planes are needed. It needs no 2D imaging
and no calibration pattern. Its related-work section traces planar-constraint
registration to Zhang and Pless (2004) and to kinematic calibration with planar
constraints (Zhuang, Motaghedi, Roth).

### 3.5 PlaneHEC, arXiv 2507.19851 (July 2025)

Read from the arXiv HTML. Eye-in-hand depth camera, arbitrary planar surface,
RANSAC plane fit, plane written as `(a, b, c, d)` with `|(a, b, c)| = 1`, planes
transformed between frames as homogeneous vectors, closed-form rotation from normals
via SVD and translation from a non-homogeneous linear system, then Gauss-Newton
refinement on the Lie algebra. The arXiv comments field says "Accepted by 2025
IEEE International Conference on Robotics & Automation (ICRA)". It postdates the spec, so it is
relevant as evidence of independent contemporaneous work, not as prior art.

### 3.6 Aisin Seiki, US 7,822,571 B2 (Kakinami)

Read from the Google Patents record. "The calibration target ... includes a first
plane surface and a second plane surface, which differs from the first plane
surface"; the device "calculates normal vectors for each surface, and determines the
sensor's rotational displacement from these vectors and the target's known
posture". Translation in a further embodiment from the intersection of three planes.
Range data only. This is rotation-from-normals for a range sensor, in a vehicle
context.

### 3.7 Wan and Song, Frontiers in Robotics and AI 2020

Read from the PMC record. Fixed Photoneo MotionCam-3D on a truss looking down; the
robot presents the flange at several poses; the flange plane is fitted by RANSAC and
the flange's outer circle is fitted with its known radius; the circle centers give
point correspondences solved by Umeyama's closed form. This is the spec's physical
arrangement (fixed 3D sensor, planar feature carried by the robot, no target
markers, no 2D) but with a point-based solve. Combining it with ref. 1's plane-based
solve is a small step.

### 3.8 Other references examined and set aside

- TU Darmstadt, EP 2 728 374 A1 (priority 30 Oct 2012), "hand-eye calibration of
  cameras, in particular depth image cameras": depth-only, marker-free registration
  of a robot-mounted depth camera, but with a smooth asymmetric body and ICP, not
  planes. Relevant only to the "no 2D markers" claim.
- Kahn, Haumann, Willert, VISAPP 2014, DOI 10.5220/0004668604810489, "Hand-eye
  calibration with a depth camera: 2D or 3D?": the 3D variant registers depth data
  to a 3D model of a calibration object by ICP; compares against 2D checkerboard
  registration. Not plane-based.
- Sharifzadeh, Biro, Kinnell, Robotics and Computer-Integrated Manufacturing 61,
  2020, DOI 10.1016/j.rcim.2019.101823, "Robust hand-eye calibration of 2D laser
  sensors using a single-plane calibration artefact": 2D laser line sensor on the
  robot, one plane, nonlinear optimization.
- Lembono, Suarez-Ruiz, Pham, arXiv 1803.00747, 2018, "SCALAR: Simultaneous
  calibration of 2D laser and robot's kinematic parameters using three planar
  constraints": 2D laser on the robot, three fixed planes, Levenberg-Marquardt.
- ASEA, US 4,815,006 (priority 1986), "Method and device for calibrating a sensor on
  an industrial robot": an optical distance sensor on the robot and a rectangular
  calibration plate; the plate normal is used to align the sensor axis by wrist
  rotations. A procedural, single-point-sensor method; relevant as early use of a
  plate's normal for sensor registration.
- Zhuang, Motaghedi, Roth, "Robot calibration with planar constraints", ICRA 1999,
  DOI 10.1109/ROBOT.1999.770073:
  kinematic calibration, not sensor registration; the origin of the planar-constraint
  formulation in the robotics literature, cited by ref. 4.

---

## 4. What a novelty argument would have to rest on

If the author wants to pursue the novelty question further, the distinguishing
features that survive the references above are narrow:

1. The target plane is known in the flange frame by a single scalar (plate
   thickness), so the solution has no plane unknowns and is closed-form in two
   decoupled steps (rotation from normals; translation, and optionally scale, from
   offsets without using the rotation estimate). Refs. 2, 3, 4 estimate the plane;
   ref. 1 knows the planes in the article frame; ref. 5 is closed-form but
   eye-in-hand and later.
2. The sensor is fixed and the plate moves (refs. 1 to 5 move the sensor; ref. 7 is
   fixed-sensor but point-based).
3. The similarity extension (uniform scale) is not in any of the references found,
   though it is a textbook extension of ref. 1's linear translation solve.

None of these is a different kind of measurement or computation from the prior art.
A patent search by a professional, including non-English patents (Chinese
applications on "hand-eye calibration" with plane fitting are numerous and were only
sampled here) and the USPTO full-text database, would be needed before relying on
any of the three.

---

## 5. Uncertainties

- Ref. 3 was read from its abstract only.
- Ref. 1's and ref. 6's claim text was read from Google Patents summaries of the
  granted claims, not the official PDF; claim scope should be checked on the
  official record.
- PlaneHEC's ICRA acceptance is from the authors' arXiv comments field, not the
  proceedings.
- Chinese-language patents were sampled through Google Patents' English
  translations only; several titles in the results (for example CN 118893631 A on a
  line-laser hand-eye method) were not read.
- The MERL patent US 9,183,631 was not read.
