"""Numerical verification of "Registration to Plane: A Cookbook" (G. N. Haven, rev. 16 June 2025).

Every numbered check below corresponds to a claim in REVIEW.md.  The script builds a
synthetic ground truth (a sensor-to-world transform X, a target plane o fixed to the
robot flange, and a series of flange poses T_k), generates the sensor-frame plane
measurements the spec assumes, and then runs the two solution methods of the spec
(Section 2.2 linear 12-unknown least squares; Section 3 rotation-by-SVD plus
translation least squares) and several deliberately wrong variants.

Conventions (identical to the spec):
    plane  v = (a, b, c, d),   point p = (x, y, z, 1),   v . p = 0
    X : sensor -> world,  X . p_s = p_w                       (spec Eq. 6)
    T_k : tool -> world (assumed; the spec does not say)      (spec Eq. 7)
    planes transform by the inverse transpose,  v' = T^{-t} v (spec Eq. 36)
    Xi = X^{-t},  Tau_k = T_k^{-t}                            (spec Section 2)

Run:  python3 verify_registration.py
All tolerances are named constants below; nothing numeric is hidden in the body.
"""
import numpy as np

from registration_model import *  # noqa: F401,F403  (parameters, helpers, solvers)

# ----------------------------------------------------------------------------- ground truth
X_true = rigid(random_rotation(),
               np.r_[rng.uniform(-SENSOR_LATERAL_HALF_RANGE_MM, SENSOR_LATERAL_HALF_RANGE_MM, size=2),
                     SENSOR_HEIGHT_ABOVE_WORKSPACE_MM])
o_simple = np.array([0.0, 0.0, 1.0, 0.0])                                        # spec Eq. 2
o_general = normalize_plane(np.array([0.21, -0.13, 0.97, -PLATE_OFFSET_SIGMA_MM]),
                            sign_rule=None)                                        # spec Eq. 4
poses = make_poses(NUM_POSES_DEFAULT)

results = {}


def check(name, ok, detail=""):
    results[name] = ok
    print(f"[{'PASS' if ok else 'FAIL'}] {name}{(': ' + detail) if detail else ''}")


print("=" * 78)
print("Check 1: Section 4, planes transform by the inverse transpose (Eq. 36)")
T = poses[0]
v = normalize_plane(rng.normal(size=4))
# three points on the plane v
basis = np.linalg.svd(v[:3][None, :])[2][1:]          # two vectors orthogonal to the normal
pts = [np.r_[-v[3] * v[:3] + basis[0] * 7 + basis[1] * 3, 1],
       np.r_[-v[3] * v[:3] - basis[0] * 2 + basis[1] * 9, 1],
       np.r_[-v[3] * v[:3] + basis[0] * 4 - basis[1] * 5, 1]]
v_new = inv_transpose(T) @ v
resid = max(abs(v_new @ (T @ p)) for p in pts)
check("1a Eq.36: transformed points satisfy transformed plane", resid < EXACT_TOL, f"max residual {resid:.2e}")
cf = plane_transform_closed_form(T[:3, :3], T[:3, 3])
check("1b closed form (T^-1)^t = [[R,0],[-t^T R,1]]", np.allclose(cf, inv_transpose(T), atol=EXACT_TOL))
check("1c inverse transpose preserves |normal|", abs(np.linalg.norm(v_new[:3]) - 1) < EXACT_TOL)

print("=" * 78)
print("Check 2: structure of Xi = X^{-t} (Eq. 16): top-right 3x1 block zero, xi44 = 1")
Xi_true = inv_transpose(X_true)
check("2a xi14=xi24=xi34=0 and xi44=1", np.allclose(Xi_true[:3, 3], 0, atol=EXACT_TOL) and abs(Xi_true[3, 3] - 1) < EXACT_TOL)
check("2b top-left 3x3 of Xi is R_X itself (not R_X^-1)", np.allclose(Xi_true[:3, :3], X_true[:3, :3], atol=EXACT_TOL))
check("2c bottom row of Xi is -s^T R_X", np.allclose(Xi_true[3, :3], -X_true[:3, 3] @ X_true[:3, :3], atol=EXACT_TOL))

for label, o in (("simple o=(0,0,1,0)", o_simple), ("general o", o_general)):
    print("=" * 78)
    print(f"Check 3 [{label}]: noise-free recovery by both methods")
    world_planes = [world_plane(T_k, o) for T_k in poses]
    sensor_planes = [sensor_plane_exact(X_true, T_k, o) for T_k in poses]
    # Eq. 7 holds exactly when both sides use the same scale and sign of o:
    lhs = [inv_transpose(X_true) @ ps for ps in sensor_planes]
    check(f"3a Eq.7 X^-t p_s = T_k^-t o holds exactly [{label}]",
          max(np.abs(l - w).max() for l, w in zip(lhs, world_planes)) < EXACT_TOL)
    Xi, X_lin, condP = solve_linear_12(sensor_planes, world_planes)
    e_rot, e_tr = transform_error(X_lin, X_true)
    check(f"3b Section 2.2 linear-12 recovers X [{label}]", e_rot < EXACT_ROT_TOL_DEG and e_tr < EXACT_TRANS_TOL_MM,
          f"rot err {e_rot:.2e} deg, trans err {e_tr:.2e} mm, cond(P) = {condP:.1f}")
    X_s3 = solve_section3(sensor_planes, world_planes)
    e_rot, e_tr = transform_error(X_s3, X_true)
    check(f"3c Section 3 (Procrustes + translation LS) recovers X [{label}]", e_rot < EXACT_ROT_TOL_DEG and e_tr < EXACT_TRANS_TOL_MM,
          f"rot err {e_rot:.2e} deg, trans err {e_tr:.2e} mm")
    # The 12-unknown system is 4 copies of one N x 3 matrix [n_k^T]:
    A = np.array([ps[:3] for ps in sensor_planes])
    Xi_rows = np.zeros((4, 3))
    for i in range(4):
        rhs = np.array([pw[i] - (ps[3] if i == 3 else 0.0) for ps, pw in zip(sensor_planes, world_planes)])
        Xi_rows[i], *_ = np.linalg.lstsq(A, rhs, rcond=None)
    check(f"3d 12-unknown LS == four 3-unknown LS with the same N x 3 matrix [{label}]",
          np.allclose(Xi_rows, Xi[:, :3], atol=1e-8))

print("=" * 78)
print("Check 4: what the spec leaves unsaid -- homogeneous scale and sign of the sensor planes")
o = o_simple
world_planes = [world_plane(T_k, o) for T_k in poses]
sensor_planes = [sensor_plane_exact(X_true, T_k, o) for T_k in poses]
scales = rng.uniform(0.5, 2.0, size=len(poses))
scaled = [ps * sc for ps, sc in zip(sensor_planes, scales)]
e_rot, e_tr = transform_error(solve_linear_12(scaled, world_planes)[1], X_true)
check("4a un-normalized sensor planes break the linear method", e_rot > 1.0 or e_tr > 1.0, f"rot err {e_rot:.1f} deg, trans err {e_tr:.1f} mm")
e_rot, e_tr = transform_error(solve_section3(scaled, world_planes), X_true)
check("4b un-normalized sensor planes break the Section 3 method", e_tr > 1.0, f"rot err {e_rot:.2e} deg, trans err {e_tr:.1f} mm")
flips = np.where(rng.uniform(size=len(poses)) < 0.5, -1.0, 1.0)
flipped = [ps * f for ps, f in zip(sensor_planes, flips)]
e_rot, e_tr = transform_error(solve_section3(flipped, world_planes), X_true)
check("4c inconsistent normal signs break the Section 3 method", e_rot > 1.0, f"rot err {e_rot:.1f} deg, trans err {e_tr:.1f} mm")
# A fixed sign rule repairs it when the plate's o-normal faces the sensor in every pose:
sensor_side = [ (w[:3] @ X_true[:3, 3] + w[3]) for w in world_planes ]   # signed distance of sensor origin
check("4d with o's normal facing the sensor, the sensor origin is on the positive side of every world plane",
      all(sd > 0 for sd in sensor_side), f"min signed distance {min(sensor_side):.1f} mm")
repaired = [normalize_plane(ps) for ps in flipped]
e_rot, e_tr = transform_error(solve_section3(repaired, world_planes), X_true)
check("4e normalizing to |n|=1, d>0 repairs 4a-4c", e_rot < EXACT_ROT_TOL_DEG and e_tr < EXACT_TRANS_TOL_MM)

print("=" * 78)
print("Check 5: the printed typo in Eq. 28 (s_y appears twice, s_z is missing)")
s_typo, _ = solve_translation(world_planes, sensor_planes, typo_sy_twice=True)
err = np.linalg.norm(s_typo - X_true[:3, 3])
check("5a Eq.28 as printed gives the wrong translation", err > 1.0, f"error {err:.1f} mm")

print("=" * 78)
print("Check 6: minimum data and degenerate pose sets")
few = make_poses(3)
sp = [sensor_plane_exact(X_true, T_k, o) for T_k in few]
wp = [world_plane(T_k, o) for T_k in few]
e_rot, e_tr = transform_error(solve_section3(sp, wp), X_true)
check("6a N = 3 poses with independent normals suffice (exact)", e_rot < EXACT_ROT_TOL_DEG and e_tr < EXACT_TRANS_TOL_MM,
      f"rot err {e_rot:.2e} deg, trans err {e_tr:.2e} mm")
two = few[:2]
_, A2 = solve_translation(wp[:2], sp[:2])
check("6b N = 2 poses: translation matrix rank 2 (one direction undetermined)", np.linalg.matrix_rank(A2) == 2)
R2, _ = solve_rotation_procrustes([p[:3] for p in sp[:2]], [p[:3] for p in wp[:2]])
check("6c N = 2 non-parallel normals still determine the rotation", rotation_angle_deg(R2.T @ X_true[:3, :3]) < EXACT_ROT_TOL_DEG)
parallel = make_poses(NUM_POSES_DEFAULT, tilt_half_range_deg=0.0)       # only spin + translation
spp = [sensor_plane_exact(X_true, T_k, o) for T_k in parallel]
wpp = [world_plane(T_k, o) for T_k in parallel]
_, Ap = solve_translation(wpp, spp)
check("6d translation/spin-only poses (parallel normals): rank 1, unsolvable", np.linalg.matrix_rank(Ap, tol=1e-9) == 1)
Xi_p, X_p, cond_p = solve_linear_12(spp, wpp)
check("6e ... and the linear method's P is rank deficient (X not recoverable)", cond_p > 1e8 or X_p is None, f"cond(P) = {cond_p:.2e}, X recovered: {X_p is not None}")

print("=" * 78)
print("Check 7: noise -- Section 2.2 vs Section 3, and centered-Kabsch vs uncentered Procrustes")
stats = {"linear12": [], "sec3": [], "sec3_centered": []}
for _ in range(NOISE_TRIALS):
    spn = [normalize_plane(add_plane_noise(ps, NORMAL_NOISE_RAD, OFFSET_NOISE_MM)) for ps in sensor_planes]
    Xi, X_lin, _ = solve_linear_12(spn, world_planes)
    stats["linear12"].append(transform_error(X_lin, X_true))
    stats["sec3"].append(transform_error(solve_section3(spn, world_planes), X_true))
    stats["sec3_centered"].append(transform_error(solve_section3(spn, world_planes, center=True), X_true))
    orth = np.linalg.norm(Xi[:3, :3].T @ Xi[:3, :3] - np.eye(3))
for key, vals in stats.items():
    vals = np.array(vals)
    print(f"   {key:14s}: rot RMS {np.sqrt((vals[:, 0] ** 2).mean()):.4f} deg, trans RMS {np.sqrt((vals[:, 1] ** 2).mean()):.3f} mm")
print(f"   last linear-12 solve: |R^T R - I| = {orth:.2e}  (the rotation block is not orthogonal)")
lin = np.array(stats["linear12"]); s3 = np.array(stats["sec3"]); s3c = np.array(stats["sec3_centered"])
check("7a Section 3 translation error <= linear-12 translation error (RMS)",
      np.sqrt((s3[:, 1] ** 2).mean()) <= np.sqrt((lin[:, 1] ** 2).mean()) * 1.0001)
check("7b centered Kabsch on normals is worse than uncentered Procrustes (RMS rot)",
      np.sqrt((s3c[:, 0] ** 2).mean()) > np.sqrt((s3[:, 0] ** 2).mean()))
check("7c linear-12 rotation block is not orthogonal under noise", orth > 1e-6, f"|R^T R - I| = {orth:.2e}")

print("=" * 78)
print("Check 8: translation does not depend on the rotation estimate at all (Eq. 27)")
s_a, _ = solve_translation(world_planes, sensor_planes)
check("8a s solves from world normals and offsets only", np.linalg.norm(s_a - X_true[:3, 3]) < 1e-6)

print("=" * 78)
print("Check 9: pose-design sensitivity -- translation RMS vs tilt half-range (noisy, Section 3)")
tilt_table = []
for tilt in (2.0, 5.0, 10.0, 20.0, 35.0, 60.0):
    errs, smins = [], []
    for _ in range(NOISE_TRIALS // 4):
        ps_k = make_poses(NUM_POSES_DEFAULT, tilt_half_range_deg=tilt)
        wp_k = [world_plane(T_k, o) for T_k in ps_k]
        sp_k = [normalize_plane(add_plane_noise(sensor_plane_exact(X_true, T_k, o), NORMAL_NOISE_RAD, OFFSET_NOISE_MM)) for T_k in ps_k]
        errs.append(transform_error(solve_section3(sp_k, wp_k), X_true))
        smins.append(np.linalg.svd(np.array([w[:3] for w in wp_k]), compute_uv=False)[-1])
    errs = np.array(errs)
    tilt_table.append((tilt, np.sqrt((errs[:, 0] ** 2).mean()), np.sqrt((errs[:, 1] ** 2).mean()), float(np.mean(smins))))
    print(f"   tilt +/-{tilt:4.0f} deg: rot RMS {tilt_table[-1][1]:.4f} deg, trans RMS {tilt_table[-1][2]:.3f} mm, mean smallest sing. value of [m_k] {tilt_table[-1][3]:.3f}")
check("9a translation error falls monotonically with tilt range", all(tilt_table[i][2] > tilt_table[i + 1][2] for i in range(len(tilt_table) - 1)))
np.save("tilt_table.npy", np.array(tilt_table))

print("=" * 78)
print("Check 10: similarity extension (rotation + translation + uniform scale, no shear)")
TRUE_SCALE = 1.02                          # sensor length unit is 2% long relative to the robot's
R_t, s_t = X_true[:3, :3], X_true[:3, 3]
sim_planes = [sensor_plane_exact_similarity(R_t, s_t, TRUE_SCALE, T_k, o) for T_k in poses]
X_sim = solve_similarity(sim_planes, world_planes)
scale_est = np.cbrt(np.linalg.det(X_sim[:3, :3]))
e_rot = rotation_angle_deg((X_sim[:3, :3] / scale_est).T @ R_t)
check("10a similarity solve recovers R, s and scale exactly", e_rot < EXACT_ROT_TOL_DEG
      and np.linalg.norm(X_sim[:3, 3] - s_t) < EXACT_TRANS_TOL_MM and abs(scale_est - TRUE_SCALE) < 1e-9,
      f"scale {scale_est:.12f}")
X_rig = solve_section3(sim_planes, world_planes)
e_rot, e_tr = transform_error(X_rig, X_true)
check("10b the rigid solve on scaled data gets R right but s wrong", e_rot < EXACT_ROT_TOL_DEG and e_tr > 1.0,
      f"trans err {e_tr:.1f} mm")
_, _, A4 = solve_translation_and_scale(world_planes[:3], sim_planes[:3])
check("10c similarity needs >= 4 poses (N = 3 gives a rank-3 N x 4 system)", np.linalg.matrix_rank(A4) == 3)
_, _, A4 = solve_translation_and_scale(world_planes[:4], sim_planes[:4])
check("10d N = 4 generic poses suffice", np.linalg.matrix_rank(A4) == 4)

print("=" * 78)
n_fail = sum(1 for v in results.values() if not v)
print(f"{len(results) - n_fail} passed, {n_fail} failed")
