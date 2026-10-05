# plane_plane_registration

Registration of a fixed 3D sensor into a robot's base frame from plane-plane
correspondences: a flat board on the robot flange is viewed at many poses, the
sensor's measured plane at each pose is matched to the board plane carried by the
robot's reported pose, and the sensor-to-base transform follows in closed form.

Terminology: "registration" is the determination of the sensor-to-external-frame
transform (extrinsic parameters); "calibration" is the determination of sensor
intrinsics.

## Contents

- `archive/` holds the source specification, "Registration to Plane: A Cookbook"
  (G. Neil Haven, June 2025), with its equations extracted to linear text.
- `docs/registration_review/REVIEW.md` evaluates the specification for clarity,
  completeness and correctness, with a NumPy script that verifies every finding,
  the author's decisions (Section 9), and `PRIOR_ART.md`, a prior-art search on
  the method's claimed novelty.
- `include/`, `src/`, `tests/`, `CMakeLists.txt`: the C++ library (rigid and
  similarity models, outlier rejection, residual diagnostics) with its GoogleTest
  suite; fixtures are generated from the Python reference model.
- `python/planereg/`: the Python tools for gathering and analyzing the test sample
  set: `core` (plane algebra, the reference solver, target-plane segmentation),
  `capture` (pose plan with the two approach sub-procedures, bootstrap, quick
  check) and `analysis` (registration with residual maps, session comparison,
  report, synthetic sessions). The code contract is
  `docs/design/planereg_code_design.md`.
- `docs/procedures/registration_capture_procedure.md` (and `.docx`): the
  technician's step-by-step procedure for the captures, modeled on the stage-1
  capture procedure of the sibling repository
  `depth_calibration_from_spherical_target`, whose `sphcal` package this one
  depends on for reading capture files and building manifests.

## Building and testing

C++ (requires a C++17 compiler, CMake 3.20, Eigen 3.3 or later; GoogleTest is
fetched if not installed):

```
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

Python (requires `sphcal` from the sibling repository):

```
pip install -e /path/to/depth_calibration_from_spherical_target
cd python && pip install -e ".[figures,test]" && python3 -m pytest -q
```

Procedure document: `docs/procedures/build/fill_procedure.py` fills the template
from the tools' help and an example plan; `docs/procedures/build/build_docx.sh`
builds the Word file (pandoc via `pypandoc_binary`).

## History

This material was developed on the branch `claude/elegant-bell-yt0be6` of the
repository `flexible_plane_fit` (commits 6054e7b to 9a535ad) and moved here as
one commit; that branch keeps the full development history.
