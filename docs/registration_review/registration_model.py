"""Reference model of the plane-plane registration problem, shared by verify_registration.py
(the spec checks) and generate_test_vectors.py (the C++ unit-test fixtures).

Conventions (identical to the spec "Registration to Plane: A Cookbook"):
    plane  v = (a, b, c, d),   point p = (x, y, z, 1),   v . p = 0
    X : sensor -> world,  X . p_s = p_w                       (spec Eq. 6)
    T_k : flange -> world (author's answer to question 1)      (spec Eq. 7)
    planes transform by the inverse transpose,  v' = T^{-t} v (spec Eq. 36)
    Xi = X^{-t},  Tau_k = T_k^{-t}                            (spec Section 2)
All tolerances and ranges are named constants; nothing numeric is hidden in function bodies.
"""
import numpy as np

# ----------------------------------------------------------------------------- parameters
RNG_SEED = 20250616                    # fixed seed so the printed numbers are reproducible
NUM_POSES_DEFAULT = 17                 # the spec's "for concreteness, N = 17"
TILT_HALF_RANGE_DEG = 35.0             # flange tilt drawn uniformly in +/- this range about x and y
SPIN_HALF_RANGE_DEG = 180.0            # rotation about the flange normal (adds no information)
TRANSLATION_HALF_RANGE_MM = 150.0      # flange position drawn uniformly in +/- this range
EXACT_TOL = 1e-9                       # tolerance for "algebraically exact" checks (noise-free)
EXACT_ROT_TOL_DEG = 1e-6               # rotation-angle floor reachable in double precision
EXACT_TRANS_TOL_MM = 1e-6
NORMAL_NOISE_RAD = 0.2 * np.pi / 180   # 0.2 degree noise on measured plane normals
OFFSET_NOISE_MM = 0.2                  # 0.2 mm noise on measured plane offsets
NOISE_TRIALS = 200                     # Monte-Carlo trials for the noisy comparisons
PLATE_OFFSET_SIGMA_MM = 12.0           # plate thickness sigma of spec Eq. 3
SENSOR_HEIGHT_ABOVE_WORKSPACE_MM = 1500.0   # sensor origin is on the flange's +z side in every pose
SENSOR_LATERAL_HALF_RANGE_MM = 300.0           # "thickness" sigma of spec Eq. 3 for the general-o tests

rng = np.random.default_rng(RNG_SEED)


# ----------------------------------------------------------------------------- helpers
def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def random_rotation():
    """Uniform random rotation via QR of a Gaussian matrix, det forced to +1."""
    q, r = np.linalg.qr(rng.normal(size=(3, 3)))
    q = q @ np.diag(np.sign(np.diag(r)))
    if np.linalg.det(q) < 0:
        q[:, 0] = -q[:, 0]
    return q


def rigid(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def inv_transpose(M):
    return np.linalg.inv(M).T


def plane_transform_closed_form(R, t):
    """(T^{-1})^t for T = [R t; 0 1], written out:  [[R, 0], [-t^T R, 1]]."""
    M = np.eye(4)
    M[:3, :3] = R
    M[3, :3] = -t @ R
    return M


def normalize_plane(v, sign_rule="d_positive"):
    """Scale so |(a,b,c)| = 1; then fix the sign so d > 0 (normal points toward the origin)."""
    v = np.asarray(v, dtype=float)
    v = v / np.linalg.norm(v[:3])
    if sign_rule == "d_positive" and v[3] < 0:
        v = -v
    return v


def make_poses(num_poses, tilt_half_range_deg=TILT_HALF_RANGE_DEG,
               spin_half_range_deg=SPIN_HALF_RANGE_DEG,
               translation_half_range_mm=TRANSLATION_HALF_RANGE_MM):
    """Flange poses T_k (tool -> world).  Tilt about x and y, spin about z, random position.
    The flange points its +z toward the sensor side: base orientation rot_y(pi) is not used
    because the sensor placement below is chosen to face the flange directly."""
    poses = []
    for _ in range(num_poses):
        tx = np.deg2rad(rng.uniform(-tilt_half_range_deg, tilt_half_range_deg))
        ty = np.deg2rad(rng.uniform(-tilt_half_range_deg, tilt_half_range_deg))
        sz = np.deg2rad(rng.uniform(-spin_half_range_deg, spin_half_range_deg))
        R = rot_x(tx) @ rot_y(ty) @ rot_z(sz)
        t = rng.uniform(-translation_half_range_mm, translation_half_range_mm, size=3)
        poses.append(rigid(R, t))
    return poses


def world_plane(T_k, o):
    """The target plane in world coordinates:  Tau_k . o  (spec Eq. 7 RHS)."""
    return inv_transpose(T_k) @ o


def sensor_plane_exact(X, T_k, o):
    """The plane the sensor would measure, noise-free:  p_s = X^t . Tau_k . o  (spec Eq. 26)."""
    return X.T @ world_plane(T_k, o)


def add_plane_noise(p, normal_noise_rad, offset_noise_mm):
    """Perturb a unit-normal plane: rotate the normal by a small random angle, shift d."""
    n = p[:3]
    axis = rng.normal(size=3)
    axis -= axis.dot(n) * n
    axis /= np.linalg.norm(axis)
    ang = rng.normal(scale=normal_noise_rad)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    Rn = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
    q = np.empty(4)
    q[:3] = Rn @ n
    q[3] = p[3] + rng.normal(scale=offset_noise_mm)
    return q


# ----------------------------------------------------------------------------- the two solvers
def solve_linear_12(sensor_planes, world_planes):
    """Spec Section 2.2 / Eq. 21: solve P xi = tau for the 12 free entries of Xi = X^{-t}.

    Row block k (4 rows):   row i (i = 1..3):  (a_k, b_k, c_k) . Xi[i, 0:3]            = tau_k[i]
                             row 4:            (a_k, b_k, c_k) . Xi[3, 0:3] + d_k       = tau_k[3]
    Returns Xi (4x4) and the recovered X = (Xi^t)^{-1} (spec Eq. 12), plus P's condition number.
    """
    N = len(sensor_planes)
    P = np.zeros((4 * N, 12))
    tau = np.zeros(4 * N)
    for k, (ps, pw) in enumerate(zip(sensor_planes, world_planes)):
        abc, d = ps[:3], ps[3]
        for i in range(4):
            P[4 * k + i, 3 * i:3 * i + 3] = abc
            tau[4 * k + i] = pw[i] - (d if i == 3 else 0.0)
    xi, *_ = np.linalg.lstsq(P, tau, rcond=None)
    Xi = np.eye(4)
    Xi[:, :3] = xi.reshape(4, 3)
    try:
        X = np.linalg.inv(Xi.T)
    except np.linalg.LinAlgError:
        X = None
    return Xi, X, np.linalg.cond(P)


def solve_rotation_procrustes(sensor_normals, world_normals, center=False):
    """Spec Eq. 29-30: find the rotation R with R n_k ~ m_k (n = sensor, m = world normals).
    Orthogonal Procrustes: H = sum n_k m_k^T, H = U S V^t, R = V U^t with det correction.
    `center=True` mimics a point-cloud Kabsch routine that subtracts centroids first,
    which is what a generic RigidTransform3D-style class would do; see REVIEW.md."""
    Nn = np.asarray(sensor_normals)
    Mm = np.asarray(world_normals)
    if center:
        Nn = Nn - Nn.mean(axis=0)
        Mm = Mm - Mm.mean(axis=0)
    H = Nn.T @ Mm
    U, S, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T, S


def solve_translation(world_planes, sensor_planes, typo_sy_twice=False):
    """Spec Eq. 27-28: s . m_k = p4_k - t4_k, where m_k = (t1,t2,t3)_k is the world normal.
    `typo_sy_twice=True` reproduces Eq. 28 exactly as printed (s_y in place of s_z)."""
    A = np.array([pw[:3] for pw in world_planes])
    b = np.array([ps[3] - pw[3] for ps, pw in zip(sensor_planes, world_planes)])
    if typo_sy_twice:
        A = A.copy()
        A[:, 1] = A[:, 1] + A[:, 2]   # t1*sx + t2*sy + t3*sy  ==  t1*sx + (t2 + t3)*sy
        A[:, 2] = 0.0
    s, *_ = np.linalg.lstsq(A, b, rcond=None)
    return s, A


def similarity(R, s, scale):
    """Sensor -> world similarity transform [scale*R s; 0 1] (rotation, uniform scale, no shear)."""
    M = np.eye(4)
    M[:3, :3] = scale * R
    M[:3, 3] = s
    return M


def sensor_plane_exact_similarity(R, s, scale, T_k, o):
    """Noise-free sensor plane under a similarity sensor->world map, normalized to |n| = 1.
    X^t (m, e) = (scale R^t m, s.m + e); dividing by scale gives (R^t m, (s.m + e)/scale)."""
    m, e = world_plane(T_k, o)[:3], world_plane(T_k, o)[3]
    return np.r_[R.T @ m, (s @ m + e) / scale]


def solve_translation_and_scale(world_planes, sensor_planes):
    """Similarity extension of Eq. 27:  m_k . s - d_k * scale = -e_k,  N x 4 least squares.
    Returns (s, scale, A)."""
    A = np.array([np.r_[pw[:3], -ps[3]] for ps, pw in zip(sensor_planes, world_planes)])
    b = np.array([-pw[3] for pw in world_planes])
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    return x[:3], x[3], A


def solve_similarity(sensor_planes, world_planes):
    """Rotation by uncentered Procrustes (scale-free), then translation and scale jointly."""
    R, _ = solve_rotation_procrustes([p[:3] for p in sensor_planes], [p[:3] for p in world_planes])
    s, scale, _ = solve_translation_and_scale(world_planes, sensor_planes)
    return similarity(R, s, scale)


def solve_section3(sensor_planes, world_planes, center=False):
    R, _ = solve_rotation_procrustes([p[:3] for p in sensor_planes],
                                     [p[:3] for p in world_planes], center=center)
    s, _ = solve_translation(world_planes, sensor_planes)
    return rigid(R, s)


def nearest_rotation(M):
    """Polar projection of a 3x3 matrix onto SO(3) (used only to *measure* non-orthogonal estimates)."""
    U, _, Vt = np.linalg.svd(M)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    return U @ D @ Vt


def rotation_angle_deg(R):
    """Angle of a rotation, computed from the Frobenius norm of R - I so that it stays
    accurate for tiny angles (the arccos(trace) formula floors at ~1e-6 deg in double)."""
    return np.rad2deg(2 * np.arcsin(min(1.0, np.linalg.norm(R - np.eye(3)) / (2 * np.sqrt(2)))))


def transform_error(X_est, X_true):
    """(rotation error in degrees, translation error in mm) of X_est relative to X_true."""
    dR = nearest_rotation(X_est[:3, :3]).T @ X_true[:3, :3]
    return rotation_angle_deg(dR), np.linalg.norm(X_est[:3, 3] - X_true[:3, 3])


