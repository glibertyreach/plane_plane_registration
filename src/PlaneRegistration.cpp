//
// PlaneRegistration.cpp
//
// Implementation of the plane-plane registration declared in
// include/lri/registration/PlaneRegistration.h.  The algorithm is REVIEW.md Section 6
// ("Recommended computation"), extended with the optional uniform scale of the Similarity model
// and with iterative outlier rejection.  Notation follows REVIEW.md Section 2:
//
//   n_k, d_k   the normalized sensor plane at pose k          (unit normal, offset)
//   m_k, e_k   the world plate plane at pose k                (unit normal, offset)
//   R, s, c    the sought rotation, translation and uniform scale; X = [c R  s; 0 1]
//
// The two equations that define the problem (see the header comment) are
//
//   R n_k = m_k                          (rotation)
//   s . m_k - c d_k = -e_k               (translation and scale)
//
#include "lri/registration/PlaneRegistration.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace lri::registration {

namespace {

// ------------------------------------------------------------------------------------------
// Named constants
// ------------------------------------------------------------------------------------------

// Fewest poses that determine each model.  A rigid transform has 6 unknowns and every plane
// pair supplies 2 rotation constraints and 1 translation constraint, so 3 non-parallel poses are
// the minimum (REVIEW.md Section 4.5).  The Similarity model adds the scale as a 7th unknown
// and so needs one more pose.  RegistrationParameters::minimum_pose_count can only raise these.
constexpr int kRigidMinimumPoseCount = 3;
constexpr int kSimilarityMinimumPoseCount = 4;

// Dimension of the space the planes live in; the number of components of a plane normal.
constexpr int kSpaceDimension = 3;

// The Similarity translation-and-scale system has one column per translation component plus one
// column for the scale, and the scale is the last unknown (x = (s_x, s_y, s_z, c)).
constexpr int kSimilarityColumnCount = kSpaceDimension + 1;
constexpr int kScaleColumn = kSpaceDimension;

// The Rigid model has no scale freedom: the scale is exactly this value.
constexpr double kRigidScale = 1.0;

// Value used to compare against exact zero (zero-length normals, an unobservable scale).
constexpr double kZero = 0.0;

// Degrees per radian.  A half turn is 180 degrees and pi radians.
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfTurnDegrees = 180.0;
constexpr double kDegreesPerRadian = kHalfTurnDegrees / kPi;

// An outlier ratio is a residual divided by its outlier threshold; a pose is an outlier
// candidate only when its largest ratio exceeds this limit (that is, a residual beyond its
// threshold).
constexpr double kOutlierRatioLimit = 1.0;

// Weight given to every direction pair when the caller supplies no weights.
constexpr double kDefaultWeight = 1.0;

// Diagonal entries of the correction matrix D = diag(1, 1, det(V U^t)) of the Procrustes solve
// (REVIEW.md Section 6, step 4).  The first two entries are always 1; the last is +1 when
// V U^t is already a proper rotation and -1 when it is a reflection.
constexpr double kNoCorrection = 1.0;
constexpr double kReflectionCorrection = -1.0;

// Dimensions of the linear oracle system (spec Section 2.2, Eq. 21).  Each pose contributes one
// block of kOracleRowsPerPose equations: three for the rows of Xi that carry the plane normal
// and one for the row that carries the offset.  Xi has kOracleRowCount rows of
// kSpaceDimension free entries each (its last column is the fixed (0, 0, 0, 1)), so the
// unknown vector xi has kOracleUnknownCount entries.
constexpr int kOracleRowsPerPose = 4;
constexpr int kOracleRowCount = 4;
constexpr int kOracleOffsetRow = 3;  // index of the Xi row (and equation) that carries the offset
constexpr int kOracleUnknownCount = kOracleRowCount * kSpaceDimension;
constexpr int kOracleHomogeneousColumn = 3;  // index of the fixed last column of Xi
constexpr double kOracleHomogeneousColumnEntry = 1.0;  // the fixed Xi(3, 3); the other three are zero

// ------------------------------------------------------------------------------------------
// Input validation helpers
// ------------------------------------------------------------------------------------------

// True when every component of the plane is a finite number.
bool IsFinite(const Plane& plane) {
  return plane.normal.allFinite() && std::isfinite(plane.offset);
}

// Scale the plane so its normal has unit length, without touching the sign.  Returns false (and
// leaves the plane unchanged) for a non-finite or zero-length normal or a non-finite offset.
// This is the "divide by |n|" half of NormalizePlane, shared with the target plane, whose sign
// must not be flipped (its normal points toward the sensor by construction).
bool ScaleToUnitNormal(Plane& plane) {
  if (!IsFinite(plane)) {
    return false;
  }
  const double length = plane.normal.norm();
  // A huge finite normal can overflow in norm(); treat that as non-finite too.
  if (!std::isfinite(length) || length <= kZero) {
    return false;
  }
  plane.normal /= length;
  plane.offset /= length;
  return true;
}

// True when every entry of the 4x4 homogeneous matrix of the pose is finite.
bool IsFinite(const Eigen::Isometry3d& transform) {
  return transform.matrix().allFinite();
}

// Describe the first problem found in the inputs, or return an empty string when they are all
// acceptable (REVIEW.md Section 6, "Inputs"; header: InvalidInput).  Messages name the offending
// pose by its zero-based index in `correspondences`.
std::string FindInputProblem(const std::vector<PlaneCorrespondence>& correspondences,
                             const TargetPlane& target) {
  Plane scaled_target = target.plane;
  if (!ScaleToUnitNormal(scaled_target)) {
    return "target plane has a non-finite value or a zero-length normal";
  }
  for (std::size_t k = 0; k < correspondences.size(); ++k) {
    const PlaneCorrespondence& correspondence = correspondences[k];
    const std::string pose_label = "pose " + std::to_string(k) + ": ";
    if (!IsFinite(correspondence.flange_to_world)) {
      return pose_label + "flange-to-world transform contains a non-finite value";
    }
    if (!IsFinite(correspondence.sensor_plane)) {
      return pose_label + "sensor plane contains a non-finite value";
    }
    if (correspondence.sensor_plane.normal.norm() <= kZero) {
      return pose_label + "sensor plane has a zero-length normal";
    }
  }
  return std::string();
}

// ------------------------------------------------------------------------------------------
// Per-pose data
// ------------------------------------------------------------------------------------------

// Everything the solver needs about one pose, in the REVIEW.md Section 2 notation.
struct PosePlanes {
  Eigen::Vector3d sensor_normal;  // n_k, unit
  double sensor_offset{0.0};      // d_k, sign chosen by the orientation convention
  Eigen::Vector3d world_normal;   // m_k, unit
  double world_offset{0.0};       // e_k
};

// Normalize every sensor plane (REVIEW.md Section 6, step 1) and compute the world plate plane of
// each pose (step 2).  Inputs must already have passed FindInputProblem().
std::vector<PosePlanes> BuildPosePlanes(const std::vector<PlaneCorrespondence>& correspondences,
                                        const Plane& unit_target,
                                        bool sensor_sees_positive_side) {
  std::vector<PosePlanes> poses;
  poses.reserve(correspondences.size());
  for (const PlaneCorrespondence& correspondence : correspondences) {
    Plane sensor_plane = correspondence.sensor_plane;
    NormalizePlane(sensor_plane, sensor_sees_positive_side);  // cannot fail: input validated
    const Plane world_plane = TransformPlane(correspondence.flange_to_world, unit_target);

    PosePlanes pose;
    pose.sensor_normal = sensor_plane.normal;
    pose.sensor_offset = sensor_plane.offset;
    pose.world_normal = world_plane.normal;
    pose.world_offset = world_plane.offset;
    poses.push_back(pose);
  }
  return poses;
}

// The subset of `poses` whose entry in `used` is true, in the original order.
std::vector<PosePlanes> SelectUsed(const std::vector<PosePlanes>& poses,
                                   const std::vector<bool>& used) {
  std::vector<PosePlanes> selected;
  for (std::size_t k = 0; k < poses.size(); ++k) {
    if (used[k]) {
      selected.push_back(poses[k]);
    }
  }
  return selected;
}

// ------------------------------------------------------------------------------------------
// Conditioning gates (REVIEW.md Section 6, step 3, and Section 4.5), measured as spreads
// (smallest singular value divided by sqrt(N); see Spread())
// ------------------------------------------------------------------------------------------

// The "spread" of a set of N rows: the smallest singular value of the N-row matrix divided by
// sqrt(N).  The singular values of a matrix of N unit-scale rows grow like sqrt(N), so dividing
// by sqrt(N) makes the measure independent of the number of poses.  It is zero when the columns
// are linearly dependent and measures how well the rows span every direction.  For three
// mutually orthogonal unit normals it is 1/sqrt(3).
double Spread(const Eigen::MatrixXd& matrix) {
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);  // singular values only: no U, V
  return svd.singularValues().minCoeff() / std::sqrt(static_cast<double>(matrix.rows()));
}

// The N x 3 matrix M whose rows are the unit world normals m_k^t.  It is the coefficient matrix
// of the translation problem and the object whose conditioning is the "normal spread".
Eigen::MatrixXd WorldNormalMatrix(const std::vector<PosePlanes>& poses) {
  Eigen::MatrixXd matrix(static_cast<Eigen::Index>(poses.size()), kSpaceDimension);
  for (std::size_t k = 0; k < poses.size(); ++k) {
    matrix.row(static_cast<Eigen::Index>(k)) = poses[k].world_normal.transpose();
  }
  return matrix;
}

// The N x 4 coefficient matrix [m_k^t, -d_k] of the Similarity translation-and-scale problem.
Eigen::MatrixXd SimilarityMatrix(const std::vector<PosePlanes>& poses) {
  Eigen::MatrixXd matrix(static_cast<Eigen::Index>(poses.size()), kSimilarityColumnCount);
  for (std::size_t k = 0; k < poses.size(); ++k) {
    const Eigen::Index row = static_cast<Eigen::Index>(k);
    matrix.row(row).head<kSpaceDimension>() = poses[k].world_normal.transpose();
    matrix(row, kScaleColumn) = -poses[k].sensor_offset;
  }
  return matrix;
}

// ------------------------------------------------------------------------------------------
// One solve
// ------------------------------------------------------------------------------------------

// The outcome of one least-squares solve over the currently used poses.
struct SolveOutcome {
  RegistrationStatus status{RegistrationStatus::Success};  // Success, or DegeneratePoses
  std::string message;
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  double scale{kRigidScale};
  double normal_spread{0.0};
  double similarity_spread{0.0};  // stays 0 for the Rigid model
};

// Rigid translation (REVIEW.md Section 6, step 5): least squares M s = (d_k - e_k) with the
// rows of M the world normals, solved by column-pivoting QR.
Eigen::Vector3d SolveRigidTranslation(const std::vector<PosePlanes>& poses) {
  const Eigen::MatrixXd normals = WorldNormalMatrix(poses);
  Eigen::VectorXd right_hand_side(static_cast<Eigen::Index>(poses.size()));
  for (std::size_t k = 0; k < poses.size(); ++k) {
    right_hand_side(static_cast<Eigen::Index>(k)) = poses[k].sensor_offset - poses[k].world_offset;
  }
  return normals.colPivHouseholderQr().solve(right_hand_side);
}

// Similarity translation and scale: least squares [m_k^t, -d_k] (s, c) = -e_k, solved by
// column-pivoting QR.  The caller has already checked the system is well conditioned.  Returns
// the unknowns ordered (s_x, s_y, s_z, c).
Eigen::VectorXd SolveSimilarityTranslationAndScale(const std::vector<PosePlanes>& poses) {
  const Eigen::MatrixXd matrix = SimilarityMatrix(poses);
  Eigen::VectorXd right_hand_side(static_cast<Eigen::Index>(poses.size()));
  for (std::size_t k = 0; k < poses.size(); ++k) {
    right_hand_side(static_cast<Eigen::Index>(k)) = -poses[k].world_offset;
  }
  return matrix.colPivHouseholderQr().solve(right_hand_side);
}

// Solve for translation (and scale) after the rotation is known, filling the corresponding
// fields of `outcome`.  Marks the outcome DegeneratePoses when the scale is unobservable, the
// similarity system is badly conditioned, or the recovered scale is not positive.
void SolveTranslationStage(const std::vector<PosePlanes>& poses,
                           const RegistrationParameters& parameters,
                           SolveOutcome& outcome) {
  if (parameters.transform_model == TransformModel::Rigid) {
    outcome.translation = SolveRigidTranslation(poses);
    outcome.scale = kRigidScale;
    return;
  }

  // Similarity: first the conditioning gate, the spread of [m_k, -d_k].  The sensor offsets d_k
  // are of order the distance from sensor to target, so the last column is divided by its
  // largest magnitude to make it comparable with the unit-normal columns.
  Eigen::MatrixXd matrix = SimilarityMatrix(poses);
  const double largest_offset = matrix.col(kScaleColumn).cwiseAbs().maxCoeff();
  if (largest_offset <= kZero) {
    outcome.status = RegistrationStatus::DegeneratePoses;
    outcome.message = "all sensor offsets are zero, so the scale is unobservable";
    return;
  }
  matrix.col(kScaleColumn) /= largest_offset;
  outcome.similarity_spread = Spread(matrix);
  if (outcome.similarity_spread < parameters.minimum_similarity_spread) {
    outcome.status = RegistrationStatus::DegeneratePoses;
    outcome.message = "similarity spread " + std::to_string(outcome.similarity_spread) +
                      " is below the minimum " +
                      std::to_string(parameters.minimum_similarity_spread);
    return;
  }

  const Eigen::VectorXd unknowns = SolveSimilarityTranslationAndScale(poses);
  outcome.translation = unknowns.head<kSpaceDimension>();
  outcome.scale = unknowns(kScaleColumn);
  // `!(scale > 0)` also catches NaN.
  if (!(outcome.scale > kZero)) {
    outcome.status = RegistrationStatus::DegeneratePoses;
    outcome.message = "the recovered scale " + std::to_string(outcome.scale) + " is not positive";
  }
}

// Solve once over `poses` (all of them are to be used): the conditioning gate, the rotation and
// the translation (and scale).  REVIEW.md Section 6, steps 3 to 6.
SolveOutcome SolveOnce(const std::vector<PosePlanes>& poses,
                       const RegistrationParameters& parameters) {
  SolveOutcome outcome;

  // Step 3: the world normals must span all three directions.
  outcome.normal_spread = Spread(WorldNormalMatrix(poses));
  if (outcome.normal_spread < parameters.minimum_normal_spread) {
    outcome.status = RegistrationStatus::DegeneratePoses;
    outcome.message = "normal spread " + std::to_string(outcome.normal_spread) +
                      " is below the minimum " + std::to_string(parameters.minimum_normal_spread);
    return outcome;
  }

  // Step 4: rotation from the normals alone.
  std::vector<Eigen::Vector3d> sensor_normals;
  std::vector<Eigen::Vector3d> world_normals;
  for (const PosePlanes& pose : poses) {
    sensor_normals.push_back(pose.sensor_normal);
    world_normals.push_back(pose.world_normal);
  }
  outcome.rotation = SolveRotationBetweenDirections(sensor_normals, world_normals);

  // Step 5: translation (and scale), which does not involve R.
  SolveTranslationStage(poses, parameters, outcome);
  return outcome;
}

// ------------------------------------------------------------------------------------------
// Residuals (REVIEW.md Section 6, step 7)
// ------------------------------------------------------------------------------------------

// The angle in degrees between the rotated sensor normal R n_k and the world normal m_k
// (REVIEW.md Section 6, step 7: theta_k).
//
// REVIEW.md writes theta_k = acos(R n_k . m_k).  That is mathematically identical to
// atan2(|R n_k x m_k|, R n_k . m_k), which is used here instead because acos is badly
// conditioned near 1: a dot product that is one rounding error (about 1e-16) short of 1 already
// gives an angle of about 1.5e-8 radian (8.5e-7 degree), a noise floor comparable to the
// tolerance of exact-data checks.  The cross product keeps full relative accuracy for tiny
// angles, and atan2 needs no clamping of the argument to [-1, 1].
double NormalAngleDegrees(const Eigen::Matrix3d& rotation, const PosePlanes& pose) {
  const Eigen::Vector3d rotated_normal = rotation * pose.sensor_normal;
  const double sine = rotated_normal.cross(pose.world_normal).norm();
  const double cosine = rotated_normal.dot(pose.world_normal);
  return std::atan2(sine, cosine) * kDegreesPerRadian;
}

// The world-frame offset residual s . m_k + e_k - c d_k: zero when the transformed sensor plane
// coincides with the world plate plane.
double OffsetResidual(const SolveOutcome& solution, const PosePlanes& pose) {
  return solution.translation.dot(pose.world_normal) + pose.world_offset -
         solution.scale * pose.sensor_offset;
}

// Residuals of every pose, used or not, in input order; `used` is copied into the result.
std::vector<PoseResidual> ComputeResiduals(const std::vector<PosePlanes>& poses,
                                           const std::vector<bool>& used,
                                           const SolveOutcome& solution) {
  std::vector<PoseResidual> residuals(poses.size());
  for (std::size_t k = 0; k < poses.size(); ++k) {
    residuals[k].normal_angle_degrees = NormalAngleDegrees(solution.rotation, poses[k]);
    residuals[k].offset_residual = OffsetResidual(solution, poses[k]);
    residuals[k].used = used[k];
  }
  return residuals;
}

// ------------------------------------------------------------------------------------------
// Outlier rejection (REVIEW.md Section 6, step 7, last sentence)
// ------------------------------------------------------------------------------------------

// The used pose with the largest outlier ratio, provided that ratio exceeds kOutlierRatioLimit.
// A pose's ratio is max(normal_angle / outlier_normal_residual_degrees,
// |offset_residual| / outlier_offset_residual): how many times its worse residual exceeds its
// threshold.  Only one pose is dropped per round because a gross outlier biases the solve and
// lifts the residuals of good poses (leverage); the worst offender is the most likely culprit.
std::optional<std::size_t> FindWorstOutlier(const std::vector<PoseResidual>& residuals,
                                            const RegistrationParameters& parameters) {
  std::optional<std::size_t> worst;
  double worst_ratio = kOutlierRatioLimit;
  for (std::size_t k = 0; k < residuals.size(); ++k) {
    if (!residuals[k].used) {
      continue;
    }
    const double normal_ratio =
        residuals[k].normal_angle_degrees / parameters.outlier_normal_residual_degrees;
    const double offset_ratio =
        std::abs(residuals[k].offset_residual) / parameters.outlier_offset_residual;
    const double ratio = std::max(normal_ratio, offset_ratio);
    if (ratio > worst_ratio) {
      worst_ratio = ratio;
      worst = k;
    }
  }
  return worst;
}

// ------------------------------------------------------------------------------------------
// Aggregation
// ------------------------------------------------------------------------------------------

// Root mean square of `values`.
double RootMeanSquare(const std::vector<double>& values) {
  double sum_of_squares = 0.0;
  for (const double value : values) {
    sum_of_squares += value * value;
  }
  return std::sqrt(sum_of_squares / static_cast<double>(values.size()));
}

// Fill the RMS and maximum statistics and `poses_used` from the residuals of the used poses.
void AggregateResiduals(RegistrationResult& result) {
  std::vector<double> normal_angles;
  std::vector<double> offset_magnitudes;
  for (const PoseResidual& residual : result.residuals) {
    if (residual.used) {
      normal_angles.push_back(residual.normal_angle_degrees);
      offset_magnitudes.push_back(std::abs(residual.offset_residual));
    }
  }
  result.poses_used = static_cast<int>(normal_angles.size());
  result.rms_normal_residual_degrees = RootMeanSquare(normal_angles);
  result.max_normal_residual_degrees = *std::max_element(normal_angles.begin(), normal_angles.end());
  result.rms_offset_residual = RootMeanSquare(offset_magnitudes);
  result.max_offset_residual = *std::max_element(offset_magnitudes.begin(), offset_magnitudes.end());
}

// True when all four acceptance thresholds of the parameters are met.
bool MeetsAcceptanceThresholds(const RegistrationResult& result,
                               const RegistrationParameters& parameters) {
  return result.rms_normal_residual_degrees <= parameters.maximum_rms_normal_residual_degrees &&
         result.rms_offset_residual <= parameters.maximum_rms_offset_residual &&
         result.max_normal_residual_degrees <= parameters.maximum_normal_residual_degrees &&
         result.max_offset_residual <= parameters.maximum_offset_residual;
}

// A result carrying only a status and a message.
RegistrationResult Failure(RegistrationStatus status, const std::string& message) {
  RegistrationResult result;
  result.status = status;
  result.message = message;
  return result;
}

// The number of poses the model needs: the geometric floor of the model, raised (never lowered)
// by RegistrationParameters::minimum_pose_count.
int RequiredPoseCount(const RegistrationParameters& parameters) {
  const int model_floor = parameters.transform_model == TransformModel::Rigid
                              ? kRigidMinimumPoseCount
                              : kSimilarityMinimumPoseCount;
  return std::max(model_floor, parameters.minimum_pose_count);
}

}  // namespace

// ------------------------------------------------------------------------------------------
// Plane, TargetPlane
// ------------------------------------------------------------------------------------------

// Spec (a, b, c, d) -> normal (a, b, c) and offset d.
Plane Plane::FromCoefficients(const Eigen::Vector4d& coefficients) {
  Plane plane;
  plane.normal = coefficients.head<kSpaceDimension>();
  plane.offset = coefficients(kSpaceDimension);
  return plane;
}

// Normal and offset -> spec (a, b, c, d).
Eigen::Vector4d Plane::Coefficients() const {
  Eigen::Vector4d coefficients;
  coefficients.head<kSpaceDimension>() = normal;
  coefficients(kSpaceDimension) = offset;
  return coefficients;
}

// Spec Eq. 3: o = (0, 0, 1, -thickness).  Deliberately no validation here; a non-finite value
// is reported by RegisterSensorToWorld().
TargetPlane TargetPlane::FromPlateThickness(double plate_thickness) {
  TargetPlane target;
  target.plane.normal = Eigen::Vector3d::UnitZ();
  target.plane.offset = -plate_thickness;
  return target;
}

// Spec Eq. 4: an arbitrary plane, scaled to a unit normal (sign untouched).  If the plane cannot
// be scaled (zero or non-finite), it is stored as given so that RegisterSensorToWorld() can
// report InvalidInput.
TargetPlane TargetPlane::FromCoefficients(const Eigen::Vector4d& coefficients) {
  TargetPlane target;
  target.plane = Plane::FromCoefficients(coefficients);
  ScaleToUnitNormal(target.plane);
  return target;
}

// ------------------------------------------------------------------------------------------
// RegistrationResult, ToString
// ------------------------------------------------------------------------------------------

// X = [scale * R, s; 0 0 0 1] (header: the sensor-to-world similarity).
Eigen::Matrix4d RegistrationResult::SensorToWorld() const {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.topLeftCorner<kSpaceDimension, kSpaceDimension>() = scale * rotation;
  transform.topRightCorner<kSpaceDimension, 1>() = translation;
  return transform;
}

// The enumerator name as text, for messages and tests.
const char* ToString(RegistrationStatus status) {
  switch (status) {
    case RegistrationStatus::Success:
      return "Success";
    case RegistrationStatus::InvalidInput:
      return "InvalidInput";
    case RegistrationStatus::TooFewPoses:
      return "TooFewPoses";
    case RegistrationStatus::DegeneratePoses:
      return "DegeneratePoses";
    case RegistrationStatus::ResidualsExceedThreshold:
      return "ResidualsExceedThreshold";
  }
  // Unreachable for a valid enumerator; keeps compilers that cannot prove it quiet.
  return "Unknown";
}

// ------------------------------------------------------------------------------------------
// Building blocks
// ------------------------------------------------------------------------------------------

// REVIEW.md Section 6, step 1: divide by |n|, then choose the sign so the origin lies on the
// requested side (offset > 0 or offset < 0).  An offset of exactly zero keeps its sign.
bool NormalizePlane(Plane& plane, bool origin_on_positive_side) {
  if (!ScaleToUnitNormal(plane)) {
    return false;
  }
  const bool offset_has_wrong_sign =
      origin_on_positive_side ? (plane.offset < kZero) : (plane.offset > kZero);
  if (offset_has_wrong_sign) {
    plane.normal = -plane.normal;
    plane.offset = -plane.offset;
  }
  return true;
}

// REVIEW.md Section 6, step 2: the plane transformed by the inverse transpose of the point map
// T = [R t; 0 1], in closed form: (R n, d - t . (R n)).
Plane TransformPlane(const Eigen::Isometry3d& point_map, const Plane& plane) {
  Plane transformed;
  transformed.normal = point_map.linear() * plane.normal;
  transformed.offset = plane.offset - point_map.translation().dot(transformed.normal);
  return transformed;
}

// REVIEW.md Section 6, step 4: weighted orthogonal Procrustes on directions.
//   H = sum_k w_k from_k to_k^t,   H = U S V^t,   R = V diag(1, 1, det(V U^t)) U^t.
// With from = n_k and to = m_k this R satisfies R n_k ~ m_k.  Returns the identity when the
// inputs are inconsistent (different lengths or a weight vector of the wrong length), since the
// interface has no error channel.
Eigen::Matrix3d SolveRotationBetweenDirections(const std::vector<Eigen::Vector3d>& from,
                                               const std::vector<Eigen::Vector3d>& to,
                                               const std::vector<double>* weights) {
  if (from.size() != to.size() || (weights != nullptr && weights->size() != from.size())) {
    return Eigen::Matrix3d::Identity();
  }

  // Cross-covariance of the two direction sets.  No centroid is subtracted: directions are
  // not points (REVIEW.md Section 4.5).
  Eigen::Matrix3d cross_covariance = Eigen::Matrix3d::Zero();
  for (std::size_t k = 0; k < from.size(); ++k) {
    const double weight = (weights != nullptr) ? (*weights)[k] : kDefaultWeight;
    cross_covariance += weight * from[k] * to[k].transpose();
  }

  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(cross_covariance,
                                              Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d& u = svd.matrixU();
  const Eigen::Matrix3d& v = svd.matrixV();

  // V U^t is orthogonal with determinant +1 (a rotation) or -1 (a reflection).  Flipping the
  // direction of the singular vector with the smallest singular value (the last column of U)
  // turns a reflection into the closest rotation.
  const double determinant = (v * u.transpose()).determinant();
  const double last_entry = determinant < kZero ? kReflectionCorrection : kNoCorrection;
  const Eigen::Vector3d correction(kNoCorrection, kNoCorrection, last_entry);
  return v * correction.asDiagonal() * u.transpose();
}

// Spec Section 2.2, Eq. 21, kept as a test oracle.  Builds the (4N) x 12 system P xi = tau
// exactly as solve_linear_12 of the reference model does and solves it by SVD least squares.
// Row block k, for i = 0..2:   n_k . Xi(i, 0..2) = m_k[i]
//                for row 3:    n_k . Xi(3, 0..2) = e_k - d_k
Eigen::Matrix4d SolveLinearPlaneTransformOracle(const std::vector<Plane>& sensor_planes,
                                                const std::vector<Plane>& world_planes) {
  Eigen::Matrix4d xi_matrix = Eigen::Matrix4d::Identity();
  if (sensor_planes.size() != world_planes.size() || sensor_planes.empty()) {
    return xi_matrix;  // no error channel; an inconsistent or empty system has no solution
  }

  const Eigen::Index pose_count = static_cast<Eigen::Index>(sensor_planes.size());
  Eigen::MatrixXd system_matrix =
      Eigen::MatrixXd::Zero(kOracleRowsPerPose * pose_count, kOracleUnknownCount);
  Eigen::VectorXd tau(kOracleRowsPerPose * pose_count);
  for (Eigen::Index k = 0; k < pose_count; ++k) {
    const Plane& sensor = sensor_planes[static_cast<std::size_t>(k)];
    const Plane& world = world_planes[static_cast<std::size_t>(k)];
    for (int i = 0; i < kOracleRowsPerPose; ++i) {
      const Eigen::Index row = kOracleRowsPerPose * k + i;
      system_matrix.block<1, kSpaceDimension>(row, kSpaceDimension * i) =
          sensor.normal.transpose();
      if (i == kOracleOffsetRow) {
        tau(row) = world.offset - sensor.offset;  // the sensor offset d_k moves to the right side
      } else {
        tau(row) = world.normal(i);
      }
    }
  }

  const Eigen::VectorXd xi = system_matrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                                 .solve(tau);

  // Unpack: row i of Xi holds xi[3i .. 3i+2]; the last column is the fixed (0, 0, 0, 1).
  for (int i = 0; i < kOracleRowCount; ++i) {
    xi_matrix.block<1, kSpaceDimension>(i, 0) = xi.segment<kSpaceDimension>(kSpaceDimension * i);
  }
  xi_matrix.col(kOracleHomogeneousColumn) = Eigen::Vector4d::Zero();
  xi_matrix(kOracleHomogeneousColumn, kOracleHomogeneousColumn) = kOracleHomogeneousColumnEntry;
  return xi_matrix;
}

// ------------------------------------------------------------------------------------------
// The registration (REVIEW.md Section 6)
// ------------------------------------------------------------------------------------------
RegistrationResult RegisterSensorToWorld(const std::vector<PlaneCorrespondence>& correspondences,
                                         const TargetPlane& target,
                                         const RegistrationParameters& parameters) {
  // Validate everything before anything else, so a bad pose is reported even when there are
  // also too few poses.
  const std::string problem = FindInputProblem(correspondences, target);
  if (!problem.empty()) {
    return Failure(RegistrationStatus::InvalidInput, problem);
  }

  Plane unit_target = target.plane;
  ScaleToUnitNormal(unit_target);  // cannot fail: validated above
  const std::vector<PosePlanes> poses = BuildPosePlanes(
      correspondences, unit_target, parameters.sensor_sees_positive_side_of_target);

  const int required_poses = RequiredPoseCount(parameters);
  std::vector<bool> used(poses.size(), true);

  // The first solve plus at most `outlier_rejection_rounds` re-solves.
  const int solve_count = std::max(parameters.outlier_rejection_rounds, 0) + 1;
  RegistrationResult result;
  for (int solve_index = 0; solve_index < solve_count; ++solve_index) {
    const std::vector<PosePlanes> used_poses = SelectUsed(poses, used);
    if (static_cast<int>(used_poses.size()) < required_poses) {
      return Failure(RegistrationStatus::TooFewPoses,
                     std::to_string(used_poses.size()) + " usable poses, but " +
                         std::to_string(required_poses) + " are required");
    }

    const SolveOutcome solution = SolveOnce(used_poses, parameters);
    if (solution.status != RegistrationStatus::Success) {
      RegistrationResult failure = Failure(solution.status, solution.message);
      failure.normal_spread = solution.normal_spread;
      failure.similarity_spread = solution.similarity_spread;
      return failure;
    }

    result.rotation = solution.rotation;
    result.translation = solution.translation;
    result.scale = solution.scale;
    result.normal_spread = solution.normal_spread;
    result.similarity_spread = solution.similarity_spread;
    result.residuals = ComputeResiduals(poses, used, solution);

    // Decide whether to drop the worst outlier and go around again.
    const bool rounds_remain = solve_index + 1 < solve_count;
    if (!rounds_remain) {
      break;
    }
    const std::optional<std::size_t> worst = FindWorstOutlier(result.residuals, parameters);
    const bool would_go_below_minimum = static_cast<int>(used_poses.size()) - 1 < required_poses;
    if (!worst.has_value() || would_go_below_minimum) {
      break;  // keep the current solve
    }
    used[*worst] = false;
  }

  AggregateResiduals(result);
  if (MeetsAcceptanceThresholds(result, parameters)) {
    result.status = RegistrationStatus::Success;
    result.message = "registration succeeded";
  } else {
    result.status = RegistrationStatus::ResidualsExceedThreshold;
    result.message = "registration solved, but a residual exceeds its acceptance threshold";
  }
  return result;
}

}  // namespace lri::registration
