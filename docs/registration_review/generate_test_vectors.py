"""Generate C++ unit-test fixtures for the plane-plane registration from the reference model.

Writes tests/registration_test_vectors.generated.h.  Each case carries the inputs the C++
API takes (target plane in the flange frame, flange poses, sensor planes), the expected
outcome, and the tolerances the test must apply.  Expected transforms come from the
same reference solvers that verify_registration.py checks against the spec, so a C++
result that matches them to the stated tolerance is a faithful implementation of the
spec's Section 3 method (orthogonal Procrustes rotation + least-squares translation).

Run from this directory:  python3 generate_test_vectors.py
"""
import numpy as np
from registration_model import (rng, rigid, random_rotation, make_poses, world_plane,
                                sensor_plane_exact, normalize_plane, add_plane_noise,
                                solve_section3, solve_rotation_procrustes, solve_translation,
                                sensor_plane_exact_similarity, solve_similarity,
                                NUM_POSES_DEFAULT, PLATE_OFFSET_SIGMA_MM, NORMAL_NOISE_RAD,
                                OFFSET_NOISE_MM, SENSOR_HEIGHT_ABOVE_WORKSPACE_MM,
                                SENSOR_LATERAL_HALF_RANGE_MM)

OUTPUT_PATH = "../../tests/registration_test_vectors.generated.h"
EXACT_ROTATION_TOL_DEG = 1e-7          # noise-free cases must reproduce X to this
EXACT_TRANSLATION_TOL = 1e-6
MATCH_ROTATION_TOL_DEG = 1e-7          # noisy cases must match the reference solve to this
MATCH_TRANSLATION_TOL = 1e-6
OUTLIER_ROTATION_TOL_DEG = 1e-6        # after rejecting the bad pose the exact answer returns
OUTLIER_TRANSLATION_TOL = 1e-5
OUTLIER_OFFSET_ERROR_MM = 8.0          # injected offset error on the outlier pose
OUTLIER_NORMAL_ERROR_DEG = 3.0         # injected normal error on the outlier pose
OUTLIER_POSE_INDEX = 5
MINIMAL_POSE_COUNT = 3
TOO_FEW_POSE_COUNT = 2
UNNORMALIZED_SCALE_RANGE = (0.5, 2.0)
SIMILARITY_TRUE_SCALE = 1.02           # sensor length unit 2% long relative to the robot's
SIMILARITY_SCALE_TOL = 1e-9
SIMILARITY_MINIMAL_POSE_COUNT = 4


def residual_stats(X, sensor_planes, world_planes):
    """RMS of the per-pose normal angle (deg) and offset residual, as the C++ code defines them."""
    scale = np.cbrt(np.linalg.det(X[:3, :3]))
    R, s = X[:3, :3] / scale, X[:3, 3]
    angles, offsets = [], []
    for ps, pw in zip(sensor_planes, world_planes):
        cosang = np.clip((R @ ps[:3]) @ pw[:3], -1.0, 1.0)
        angles.append(np.rad2deg(np.arccos(cosang)))
        offsets.append(s @ pw[:3] + pw[3] - scale * ps[3])   # world-frame offset residual
    angles, offsets = np.array(angles), np.array(offsets)
    return np.sqrt((angles ** 2).mean()), np.sqrt((offsets ** 2).mean())


def make_case(name, o, poses, sensor_planes, expected_status, expected_X=None,
              rejected=(), tol_rot=EXACT_ROTATION_TOL_DEG, tol_tr=EXACT_TRANSLATION_TOL,
              outlier_rounds=0, transform_model="Rigid", expected_scale=1.0):
    world_planes = [world_plane(T, o) for T in poses]
    if expected_X is not None:
        keep = [i for i in range(len(poses)) if i not in rejected]
        rms_n, rms_o = residual_stats(expected_X, [normalize_plane(sensor_planes[i]) for i in keep],
                                      [world_planes[i] for i in keep])
    else:
        rms_n, rms_o = float("nan"), float("nan")
    return dict(name=name, o=o, poses=poses, sensor_planes=sensor_planes, status=expected_status,
                X=expected_X, rms_n=rms_n, rms_o=rms_o, rejected=list(rejected),
                tol_rot=tol_rot, tol_tr=tol_tr, outlier_rounds=outlier_rounds,
                transform_model=transform_model, expected_scale=expected_scale)


X_true = rigid(random_rotation(),
               np.r_[rng.uniform(-SENSOR_LATERAL_HALF_RANGE_MM, SENSOR_LATERAL_HALF_RANGE_MM, size=2),
                     SENSOR_HEIGHT_ABOVE_WORKSPACE_MM])
o_thickness = np.array([0.0, 0.0, 1.0, -PLATE_OFFSET_SIGMA_MM])       # spec Eq. 3, sigma = plate thickness
o_general = normalize_plane(np.array([0.21, -0.13, 0.97, -PLATE_OFFSET_SIGMA_MM]), sign_rule=None)

cases = []

poses = make_poses(NUM_POSES_DEFAULT)
exact = [sensor_plane_exact(X_true, T, o_thickness) for T in poses]
cases.append(make_case("exact_thickness_17_poses", o_thickness, poses, exact, "Success", X_true))

poses_g = make_poses(NUM_POSES_DEFAULT)
exact_g = [sensor_plane_exact(X_true, T, o_general) for T in poses_g]
cases.append(make_case("exact_general_plane_17_poses", o_general, poses_g, exact_g, "Success", X_true))

poses_3 = make_poses(MINIMAL_POSE_COUNT)
exact_3 = [sensor_plane_exact(X_true, T, o_thickness) for T in poses_3]
cases.append(make_case("exact_minimal_3_poses", o_thickness, poses_3, exact_3, "Success", X_true))

scales = rng.uniform(*UNNORMALIZED_SCALE_RANGE, size=len(poses))
flips = np.where(rng.uniform(size=len(poses)) < 0.5, -1.0, 1.0)
messy = [p * sc * f for p, sc, f in zip(exact, scales, flips)]
cases.append(make_case("exact_unnormalized_and_flipped_planes", o_thickness, poses, messy, "Success", X_true))

noisy = [add_plane_noise(p, NORMAL_NOISE_RAD, OFFSET_NOISE_MM) for p in exact]
noisy_norm = [normalize_plane(p) for p in noisy]
X_ref = solve_section3(noisy_norm, [world_plane(T, o_thickness) for T in poses])
cases.append(make_case("noisy_17_poses_matches_reference_solve", o_thickness, poses, noisy, "Success", X_ref,
                       tol_rot=MATCH_ROTATION_TOL_DEG, tol_tr=MATCH_TRANSLATION_TOL))

parallel = make_poses(NUM_POSES_DEFAULT, tilt_half_range_deg=0.0)
exact_par = [sensor_plane_exact(X_true, T, o_thickness) for T in parallel]
cases.append(make_case("degenerate_parallel_normals", o_thickness, parallel, exact_par, "DegeneratePoses"))

cases.append(make_case("too_few_poses", o_thickness, poses[:TOO_FEW_POSE_COUNT], exact[:TOO_FEW_POSE_COUNT], "TooFewPoses"))

outlier = [p.copy() for p in exact]
bad = add_plane_noise(outlier[OUTLIER_POSE_INDEX], np.deg2rad(OUTLIER_NORMAL_ERROR_DEG), 0.0)
bad[3] += OUTLIER_OFFSET_ERROR_MM
outlier[OUTLIER_POSE_INDEX] = bad
cases.append(make_case("one_outlier_pose_rejected", o_thickness, poses, outlier, "Success", X_true,
                       rejected=(OUTLIER_POSE_INDEX,), tol_rot=OUTLIER_ROTATION_TOL_DEG,
                       tol_tr=OUTLIER_TRANSLATION_TOL, outlier_rounds=3))

R_t, s_t = X_true[:3, :3], X_true[:3, 3]
sim = [sensor_plane_exact_similarity(R_t, s_t, SIMILARITY_TRUE_SCALE, T, o_thickness) for T in poses]
X_sim_true = np.eye(4); X_sim_true[:3, :3] = SIMILARITY_TRUE_SCALE * R_t; X_sim_true[:3, 3] = s_t
cases.append(make_case("exact_similarity_scale_17_poses", o_thickness, poses, sim, "Success", X_sim_true,
                       transform_model="Similarity", expected_scale=SIMILARITY_TRUE_SCALE))
cases.append(make_case("exact_similarity_minimal_4_poses", o_thickness, poses[:SIMILARITY_MINIMAL_POSE_COUNT],
                       sim[:SIMILARITY_MINIMAL_POSE_COUNT], "Success", X_sim_true,
                       transform_model="Similarity", expected_scale=SIMILARITY_TRUE_SCALE))
cases.append(make_case("similarity_too_few_3_poses", o_thickness, poses[:MINIMAL_POSE_COUNT],
                       sim[:MINIMAL_POSE_COUNT], "TooFewPoses", transform_model="Similarity"))
# The rigid model on scaled data: rotation still right, translation wrong by tens of mm (review check 10b).
# Its offset residuals (1.5 mm RMS) exceed the default acceptance threshold, so the status must say so.
cases.append(make_case("rigid_model_on_scaled_data_is_wrong", o_thickness, poses, sim, "ResidualsExceedThreshold",
                       solve_section3([normalize_plane(p) for p in sim], [world_plane(T, o_thickness) for T in poses]),
                       tol_rot=MATCH_ROTATION_TOL_DEG, tol_tr=MATCH_TRANSLATION_TOL))


def arr(values):
    return "{" + ", ".join(repr(float(v)) for v in values) + "}"


lines = ["// GENERATED by docs/registration_review/generate_test_vectors.py -- do not edit by hand.",
         "// Fixtures for tests/PlaneRegistrationTest.cpp; expected values come from the Python reference",
         "// model in docs/registration_review/registration_model.py (seeded, reproducible).",
         "#pragma once", "#include <array>", "#include <string>", "#include <vector>", "",
         "namespace lri::registration::test_vectors {", "",
         "struct PoseAndPlane {",
         "  std::array<double, 16> flange_to_world_row_major;  // T_k, flange -> world, 4x4 row-major",
         "  std::array<double, 4> sensor_plane;                 // (a, b, c, d) in sensor coordinates, as measured",
         "};", "",
         "struct Case {",
         "  std::string name;",
         "  std::array<double, 4> target_plane_in_flange;        // o = (a, b, c, d), unit normal toward the sensor",
         "  std::vector<PoseAndPlane> correspondences;",
         "  std::string expected_status;                        // \"Success\", \"ResidualsExceedThreshold\", \"TooFewPoses\", \"DegeneratePoses\"",
         "  std::array<double, 16> expected_sensor_to_world_row_major;  // X, valid only when Success",
         "  double expected_rms_normal_residual_degrees;        // over the poses that are used, NaN unless Success",
         "  double expected_rms_offset_residual;                // same units as the inputs (mm in the model)",
         "  std::vector<int> expected_rejected_pose_indices;    // poses outlier rejection must drop",
         "  int outlier_rejection_rounds;                       // parameter value the test must set",
         "  std::string transform_model;                        // \"Rigid\" or \"Similarity\"",
         "  double expected_scale;                              // 1.0 for rigid cases",
         "  double tolerance_scale;",
         "  double tolerance_rotation_degrees;                  // allowed |angle(R_est^t R_expected)|",
         "  double tolerance_translation;                       // allowed |s_est - s_expected|",
         "};", "",
         "inline const std::vector<Case>& cases() {",
         "  static const std::vector<Case> all = {"]
for c in cases:
    lines.append("    Case{")
    lines.append(f"      \"{c['name']}\",")
    lines.append(f"      {arr(c['o'])},")
    lines.append("      {")
    for T, p in zip(c["poses"], c["sensor_planes"]):
        lines.append(f"        PoseAndPlane{{{arr(T.reshape(-1))}, {arr(p)}}},")
    lines.append("      },")
    lines.append(f"      \"{c['status']}\",")
    X = c["X"] if c["X"] is not None else np.full((4, 4), np.nan)
    lines.append(f"      {arr(X.reshape(-1))},")
    lines.append(f"      {repr(float(c['rms_n']))}, {repr(float(c['rms_o']))},")
    lines.append("      {" + ", ".join(str(i) for i in c["rejected"]) + "},")
    lines.append(f"      {c['outlier_rounds']},")
    lines.append(f"      \"{c['transform_model']}\", {repr(float(c['expected_scale']))}, {repr(SIMILARITY_SCALE_TOL)},")
    lines.append(f"      {repr(c['tol_rot'])}, {repr(c['tol_tr'])},")
    lines.append("    },")
lines += ["  };", "  return all;", "}", "", "}  // namespace lri::registration::test_vectors", ""]
text = "\n".join(lines).replace("nan", "std::numeric_limits<double>::quiet_NaN()")
text = text.replace("#include <array>", "#include <array>\n#include <limits>")
import os
os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
open(OUTPUT_PATH, "w").write(text)
print(f"wrote {OUTPUT_PATH}: {len(cases)} cases")
for c in cases:
    print(f"  {c['name']:40s} status={c['status']:16s} rms_n={c['rms_n']:.4f} deg rms_o={c['rms_o']:.4f}")
