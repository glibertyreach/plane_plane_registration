#pragma once
//
// PlaneRegistration.h
//
// Registration of a sensor into the robot (world) coordinate system from plane-plane
// correspondences, following "Registration to Plane: A Cookbook" (G. N. Haven, rev. 16 June
// 2025) as corrected and completed in docs/registration_review/REVIEW.md.
//
// The problem
// -----------
// A flat target plate is fixed to the robot flange.  The robot moves the flange through a
// series of poses T_k (flange -> world, as reported by the controller).  At each pose the
// sensor measures the plate as a plane in its own coordinate system.  We seek the transform
// X that maps sensor coordinates to world coordinates.
//
// Plane convention (the spec's):  a plane is (a, b, c, d) and a point (x, y, z) lies on it when
// a x + b y + c z + d = 0.  Here a plane is stored as a unit normal n = (a, b, c) and an offset
// d; n . p + d is then the signed distance of p from the plane, positive on the side n points
// to, and the origin lies on the positive side exactly when d > 0.
//
// Under a rigid or similarity map of points, a plane's coefficients transform by the inverse
// transpose of the point map (spec Section 4).  For the plate at pose k this gives the world
// plane (m_k, e_k) = (R_k n_o, d_o - t_k . m_k) with T_k = [R_k t_k; 0 1] and o = (n_o, d_o).
// The sensor plane (n_k, d_k), once normalized, must be the same plane, which splits into
//
//     R n_k = m_k                         (rotation; spec Eq. 29)
//     s . m_k - c d_k = -e_k              (translation and scale; spec Eq. 27 with scale c)
//
// for X = [c R  s; 0 1].  The rotation is the orthogonal Procrustes solution on the normals
// (a 3x3 SVD, no centroid subtraction); the translation and, if requested, the scale are a
// small linear least-squares problem that does not involve R.  With c fixed at 1 this is the
// spec's Section 3 method.
//
// Orientation convention (author's decision, REVIEW.md Section 9): the flange +z axis points
// toward the sensor and the sensor always views the plate from that side, so every sensor
// plane is normalized to d > 0 (the sensor origin on its positive side) and the target plane
// keeps its normal along flange +z.
//
// Units: lengths are in whatever unit the robot poses and the sensor planes share (mm in the
// reference data).  Angles in this interface are in degrees.
//
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace lri::registration {

// ------------------------------------------------------------------------------------------
// Basic types
// ------------------------------------------------------------------------------------------

// A plane n . p + d = 0.  Not necessarily normalized until NormalizePlane() has been applied.
struct Plane {
  Eigen::Vector3d normal{0.0, 0.0, 1.0};
  double offset{0.0};

  // Build from the spec's (a, b, c, d) coefficient vector.
  static Plane FromCoefficients(const Eigen::Vector4d& coefficients);
  Eigen::Vector4d Coefficients() const;
};

// The target plate's plane expressed in the flange frame (the spec's o).
struct TargetPlane {
  // Usual case (spec Eq. 3): the plate's measured face lies at z = plate_thickness along the
  // flange +z axis, normal along +z, i.e. o = (0, 0, 1, -plate_thickness).
  static TargetPlane FromPlateThickness(double plate_thickness);

  // General case (spec Eq. 4): an arbitrary plane in the flange frame.  The coefficients are
  // normalized to a unit normal; the normal must already point toward the sensor's side.
  static TargetPlane FromCoefficients(const Eigen::Vector4d& coefficients);

  Plane plane;  // always stored with a unit normal
};

// One observation: where the flange was, and what the sensor saw there.
struct PlaneCorrespondence {
  Eigen::Isometry3d flange_to_world{Eigen::Isometry3d::Identity()};  // T_k from the controller
  Plane sensor_plane;  // p_k^s, any scale and either sign; normalized internally
};

enum class TransformModel {
  Rigid,       // X = [R s; 0 1]              (spec Section 3; needs >= 3 poses)
  Similarity,  // X = [c R s; 0 1], c > 0     (uniform scale, no shear; needs >= 4 poses)
};

// ------------------------------------------------------------------------------------------
// Parameters.  Every threshold is a named parameter; the defaults are placeholders derived
// from the synthetic noise model of the review (0.2 degree normal noise, 0.2 mm offset noise)
// and are expected to be revised on real-world tests (author's decision, REVIEW.md Section 9).
// ------------------------------------------------------------------------------------------
struct RegistrationParameters {
  TransformModel transform_model{TransformModel::Rigid};

  // Fewest poses accepted.  The geometry needs 3 for Rigid and 4 for Similarity; this
  // parameter can only raise that floor, never lower it.
  int minimum_pose_count{3};

  // Conditioning gate on the pose set.  The "normal spread" is the smallest singular value of
  // the N x 3 matrix whose rows are the unit world normals m_k, divided by sqrt(N); that is the
  // root-mean-square component of the normals along their least-covered direction, which does
  // not grow with the number of poses.  It is 0 when all plates are parallel (unsolvable) and
  // at most 1/sqrt(3) ~ 0.577 for normals spread uniformly over all directions.  In the review's
  // model, 17 poses tilted within +/-5 degrees give about 0.04 (and 1.5 mm of registration
  // error), +/-10 degrees about 0.08 (0.8 mm), +/-20 degrees about 0.16 (0.4 mm).
  double minimum_normal_spread{0.05};

  // For the Similarity model only: the same measure applied to the N x 4 matrix [m_k, -d_k]
  // after scaling its last column by 1/max|d_k| and dividing by sqrt(N).  The scale is
  // observable only through the variation of the plate distance d_k across poses relative to
  // the variation of the normals, so this value is small (about 0.03 for the review's model
  // with 17 poses at a 1.5 m standoff varying by +/-0.15 m); vary the standoff to raise it.
  double minimum_similarity_spread{0.01};

  // Acceptance thresholds on the final residuals.  A result outside them is still returned,
  // with status ResidualsExceedThreshold, so the caller can inspect it.
  double maximum_rms_normal_residual_degrees{0.5};
  double maximum_rms_offset_residual{0.5};
  double maximum_normal_residual_degrees{1.0};  // worst single pose
  double maximum_offset_residual{1.0};          // worst single pose

  // Outlier rejection: after a solve, the single used pose with the largest residual relative
  // to its outlier threshold (max of normal ratio and offset ratio) is dropped if that ratio
  // exceeds 1, and the solve is repeated; at most this many poses are dropped, one per round.
  // One pose per round, because a gross outlier biases the first solve and lifts the residuals
  // of good poses above the thresholds (leverage); dropping everything over threshold at once
  // discards good data.  0 disables rejection.  Rejection never reduces the used poses below
  // the pose floor.
  int outlier_rejection_rounds{0};
  double outlier_normal_residual_degrees{1.0};
  double outlier_offset_residual{1.0};

  // Orientation convention.  True: the sensor views the side of the plate that the target
  // plane's normal points to, so sensor planes are normalized to offset > 0.  False: the
  // opposite side, offset < 0.  See the file header.
  bool sensor_sees_positive_side_of_target{true};
};

// ------------------------------------------------------------------------------------------
// Results
// ------------------------------------------------------------------------------------------
enum class RegistrationStatus {
  Success,                   // solved and all residual thresholds met
  InvalidInput,              // a non-finite value or a zero-length normal in the inputs
  TooFewPoses,               // fewer poses than the model or minimum_pose_count needs
  DegeneratePoses,           // normal spread (or similarity spread) below its threshold
  ResidualsExceedThreshold,  // solved, but an acceptance threshold is violated
};

const char* ToString(RegistrationStatus status);

// Per-pose diagnostics, in the order of the input correspondences.
struct PoseResidual {
  double normal_angle_degrees{0.0};  // angle between R n_k and m_k
  double offset_residual{0.0};       // s . m_k + e_k - c d_k, in length units (world frame)
  bool used{true};                   // false once outlier rejection has dropped the pose
};

struct RegistrationResult {
  RegistrationStatus status{RegistrationStatus::InvalidInput};
  std::string message;  // human-readable explanation of the status

  // The transform, valid whenever status is Success or ResidualsExceedThreshold.
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};  // R, a proper rotation
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};   // s, the sensor origin in world coordinates
  double scale{1.0};                                      // c; exactly 1 for the Rigid model
  Eigen::Matrix4d SensorToWorld() const;                  // [scale * rotation, translation; 0 1]

  // Diagnostics.
  std::vector<PoseResidual> residuals;
  int poses_used{0};
  double normal_spread{0.0};         // normal spread of the used poses (see RegistrationParameters)
  double similarity_spread{0.0};     // similarity spread of the used poses; 0 for the Rigid model
  double rms_normal_residual_degrees{0.0};  // over the used poses
  double max_normal_residual_degrees{0.0};
  double rms_offset_residual{0.0};
  double max_offset_residual{0.0};
};

// ------------------------------------------------------------------------------------------
// The registration (spec Section 3, extended with scale and diagnostics; REVIEW.md Section 6)
// ------------------------------------------------------------------------------------------
RegistrationResult RegisterSensorToWorld(const std::vector<PlaneCorrespondence>& correspondences,
                                         const TargetPlane& target,
                                         const RegistrationParameters& parameters);

// ------------------------------------------------------------------------------------------
// Building blocks.  Public so that they can be tested in isolation and later lifted into the
// existing RigidTransform3D class (author's decision, REVIEW.md Section 9).
// ------------------------------------------------------------------------------------------

// Scale a plane to a unit normal and choose its sign so that the origin lies on the requested
// side (offset > 0 when origin_on_positive_side, offset < 0 otherwise).  An offset of exactly
// zero is left with the sign it has.  Returns false, leaving the plane unchanged, for a
// zero-length or non-finite normal or a non-finite offset.
bool NormalizePlane(Plane& plane, bool origin_on_positive_side);

// Transform a plane by the inverse transpose of a rigid point map: (R n, d - t . (R n)).
// This is the spec's Eq. 36 written in closed form; it preserves the normal's length.
Plane TransformPlane(const Eigen::Isometry3d& point_map, const Plane& plane);

// Rotation R minimizing sum_k w_k |R from_k - to_k|^2 over proper rotations (orthogonal
// Procrustes on direction vectors: H = sum w_k from_k to_k^t, H = U S V^t, R = V D U^t with
// D = diag(1, 1, det(V U^t))).  No centroid subtraction: the inputs are directions, not points.
// Weights default to 1.  Requires at least two non-parallel directions.
Eigen::Matrix3d SolveRotationBetweenDirections(const std::vector<Eigen::Vector3d>& from,
                                               const std::vector<Eigen::Vector3d>& to,
                                               const std::vector<double>* weights = nullptr);

// Section 2.2 of the spec, kept as a test oracle only: the unconstrained linear least-squares
// solve for the 12 free entries of Xi = X^{-t} from P xi = tau (spec Eq. 21), returned as the
// full 4x4 Xi.  Its 3x3 block is not constrained to be a rotation, which is why it is
// expected to be inferior to RegisterSensorToWorld under noise.  Inputs must already be
// normalized sensor planes and the matching world planes (m_k, e_k).
Eigen::Matrix4d SolveLinearPlaneTransformOracle(const std::vector<Plane>& sensor_planes,
                                                const std::vector<Plane>& world_planes);

}  // namespace lri::registration
