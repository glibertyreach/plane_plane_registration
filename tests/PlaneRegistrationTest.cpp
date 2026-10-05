//
// PlaneRegistrationTest.cpp
//
// Unit tests for lri/registration/PlaneRegistration.h.  The end-to-end cases come from the
// generated fixtures in registration_test_vectors.generated.h (expected values produced by the
// Python reference model in docs/registration_review/registration_model.py); the remaining tests
// exercise the public building blocks one at a time.
//
#include "lri/registration/PlaneRegistration.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "registration_test_vectors.generated.h"

namespace lri::registration {

// Print a fixture by name, so that test reports and ctest names stay readable (found by
// argument-dependent lookup; the fixture header itself is generated and must not be edited).
namespace test_vectors {
void PrintTo(const Case& fixture, std::ostream* stream) { *stream << fixture.name; }
}  // namespace test_vectors

namespace {

namespace tv = test_vectors;

// ------------------------------------------------------------------------------------------
// Named constants
// ------------------------------------------------------------------------------------------

// Seed of the random number generator used by the building-block tests, so that failures are
// reproducible.
constexpr unsigned int kRandomSeed = 20250616;

// "Exact" comparisons of quantities computed in double precision from well-conditioned inputs.
constexpr double kExactTolerance = 1e-9;
// Tolerance for the determinant of a returned rotation (spec: det = +1 within 1e-12).
constexpr double kDeterminantTolerance = 1e-12;
// Tolerance for comparing the oracle's Xi with the exact X^{-t} and for its orthogonality.
constexpr double kOracleTolerance = 1e-8;
// Tolerance on RMS residuals against the reference model.
constexpr double kRmsNormalToleranceDegrees = 1e-6;
constexpr double kRmsOffsetTolerance = 1e-6;

// With 0.2 degree and 0.2 mm noise the unconstrained oracle's 3x3 block departs from a rotation
// by far more than this; if it did not, the oracle would not demonstrate why the constrained
// solver is preferred.
constexpr double kNoisyOrthogonalityDefectMinimum = 1e-4;
// How close the polar projection of the oracle's 3x3 block must be to the rigid solver's rotation.
constexpr double kPolarVersusRigidToleranceDegrees = 0.1;

// Degrees per radian and the half turn used for rotation-error reporting.
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesPerRadian = 180.0 / kPi;

// Rotation error: angle = 2 asin(|R_est^t R_exp - I|_F / (2 sqrt 2)); the denominator is the
// Frobenius norm of R - I for a rotation by a half turn, so the ratio lies in [0, 1].
constexpr double kFrobeniusNormOfHalfTurnMinusIdentity = 2.0 * 1.4142135623730951;
constexpr double kHalfAngleFactor = 2.0;

// Number of sample directions and the magnitude range used by the random-direction tests.
constexpr int kThreeDirections = 3;
constexpr int kTwoDirections = 2;
constexpr int kCoplanarDirectionCount = 5;
constexpr int kWeightedDirectionCount = 6;
constexpr double kMaxRandomAngle = 3.0;           // radians; rotation vector magnitude bound
constexpr double kRandomTranslationBound = 100.0;  // length units
constexpr double kMinimumCorruptionError = 1e-3;  // unweighted solve must be visibly wrong

// Plane normalization test inputs.
constexpr double kScaleFactor = 5.0;  // a normal of length 5 before normalization
constexpr double kOffsetBeforeScaling = 10.0;
constexpr double kOffsetAfterScaling = kOffsetBeforeScaling / kScaleFactor;

// Tolerance for the spread of exactly orthogonal normals, which is 1/sqrt(3) in exact arithmetic.
constexpr double kSpreadTolerance = 1e-12;
constexpr int kOrthogonalNormalCount = 3;
// Plate thickness chosen negative so that, with the identity sensor-to-world map and flange
// translations of zero, the world plate offset e_k = -thickness is positive and the sensor
// planes need no sign flip when normalized to d > 0.
constexpr double kPositiveOffsetThickness = -12.0;
// Quarter turn used to point the flange +z axis along the world x and y axes.
constexpr double kQuarterTurnRadians = kPi / 2.0;

// Parameter variants.
constexpr int kOutlierRoundsForOutlierCase = 3;
constexpr int kInvalidPoseIndex = 2;

// ------------------------------------------------------------------------------------------
// Fixture helpers
// ------------------------------------------------------------------------------------------

// Row-major 16 doubles -> 4x4 matrix.
Eigen::Matrix4d MatrixFromRowMajor(const std::array<double, 16>& values) {
  return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(values.data());
}

// Row-major 16 doubles -> rigid transform (flange to world).
Eigen::Isometry3d IsometryFromRowMajor(const std::array<double, 16>& values) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.matrix() = MatrixFromRowMajor(values);
  return transform;
}

// (a, b, c, d) -> Plane.
Plane PlaneFromArray(const std::array<double, 4>& values) {
  return Plane::FromCoefficients(Eigen::Vector4d(values[0], values[1], values[2], values[3]));
}

// The case's correspondences in the library's types.
std::vector<PlaneCorrespondence> CorrespondencesOf(const tv::Case& fixture) {
  std::vector<PlaneCorrespondence> correspondences;
  for (const tv::PoseAndPlane& entry : fixture.correspondences) {
    PlaneCorrespondence correspondence;
    correspondence.flange_to_world = IsometryFromRowMajor(entry.flange_to_world_row_major);
    correspondence.sensor_plane = PlaneFromArray(entry.sensor_plane);
    correspondences.push_back(correspondence);
  }
  return correspondences;
}

// The case's target plane.
TargetPlane TargetOf(const tv::Case& fixture) {
  return TargetPlane::FromCoefficients(Eigen::Vector4d(fixture.target_plane_in_flange[0],
                                                       fixture.target_plane_in_flange[1],
                                                       fixture.target_plane_in_flange[2],
                                                       fixture.target_plane_in_flange[3]));
}

// Registration parameters for the case: defaults except the model and the rejection rounds.
RegistrationParameters ParametersOf(const tv::Case& fixture) {
  RegistrationParameters parameters;
  parameters.transform_model =
      fixture.transform_model == "Similarity" ? TransformModel::Similarity : TransformModel::Rigid;
  parameters.outlier_rejection_rounds = fixture.outlier_rejection_rounds;
  return parameters;
}

// Find a fixture by name; fails the test if absent.
const tv::Case& FixtureNamed(const char* name) {
  for (const tv::Case& fixture : tv::cases()) {
    if (fixture.name == name) {
      return fixture;
    }
  }
  ADD_FAILURE() << "no fixture named " << name;
  return tv::cases().front();
}

// The expected X split into rotation, translation and scale; X = [scale * R, s; 0 1].
struct ExpectedTransform {
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  double scale;
};

ExpectedTransform SplitSimilarity(const Eigen::Matrix4d& transform) {
  const Eigen::Matrix3d linear = transform.topLeftCorner<3, 3>();
  ExpectedTransform split;
  split.scale = std::cbrt(linear.determinant());  // det(scale R) = scale^3
  split.rotation = linear / split.scale;
  split.translation = transform.topRightCorner<3, 1>();
  return split;
}

// Angle in degrees of the rotation R_est^t R_expected, accurate for tiny angles.
double RotationErrorDegrees(const Eigen::Matrix3d& estimated, const Eigen::Matrix3d& expected) {
  const Eigen::Matrix3d difference = estimated.transpose() * expected - Eigen::Matrix3d::Identity();
  const double ratio = std::min(1.0, difference.norm() / kFrobeniusNormOfHalfTurnMinusIdentity);
  return kHalfAngleFactor * std::asin(ratio) * kDegreesPerRadian;
}

// Sensor planes normalized to d > 0 and the matching world planes, for the oracle tests.
void BuildOraclePlanes(const tv::Case& fixture, std::vector<Plane>& sensor_planes,
                       std::vector<Plane>& world_planes) {
  const Plane target = TargetOf(fixture).plane;
  for (const tv::PoseAndPlane& entry : fixture.correspondences) {
    Plane sensor = PlaneFromArray(entry.sensor_plane);
    EXPECT_TRUE(NormalizePlane(sensor, true));
    sensor_planes.push_back(sensor);
    world_planes.push_back(TransformPlane(IsometryFromRowMajor(entry.flange_to_world_row_major), target));
  }
}

// A reproducible random unit vector.
Eigen::Vector3d RandomUnitVector(std::mt19937& generator) {
  std::normal_distribution<double> gaussian;
  Eigen::Vector3d vector(gaussian(generator), gaussian(generator), gaussian(generator));
  return vector.normalized();
}

// A reproducible random proper rotation.
Eigen::Matrix3d RandomRotation(std::mt19937& generator) {
  std::uniform_real_distribution<double> angle(-kMaxRandomAngle, kMaxRandomAngle);
  const Eigen::AngleAxisd rotation(angle(generator), RandomUnitVector(generator));
  return rotation.toRotationMatrix();
}

// ------------------------------------------------------------------------------------------
// Fixture-driven end-to-end tests
// ------------------------------------------------------------------------------------------

class FixtureTest : public ::testing::TestWithParam<tv::Case> {};

TEST_P(FixtureTest, MatchesReferenceModel) {
  const tv::Case& fixture = GetParam();
  const RegistrationResult result =
      RegisterSensorToWorld(CorrespondencesOf(fixture), TargetOf(fixture), ParametersOf(fixture));

  ASSERT_STREQ(ToString(result.status), fixture.expected_status.c_str()) << result.message;
  const bool has_transform =
      fixture.expected_status == "Success" || fixture.expected_status == "ResidualsExceedThreshold";
  if (!has_transform) {
    return;  // TooFewPoses and DegeneratePoses carry no transform to compare
  }

  const ExpectedTransform expected =
      SplitSimilarity(MatrixFromRowMajor(fixture.expected_sensor_to_world_row_major));
  EXPECT_LE(RotationErrorDegrees(result.rotation, expected.rotation),
            fixture.tolerance_rotation_degrees);
  EXPECT_LE((result.translation - expected.translation).norm(), fixture.tolerance_translation);
  EXPECT_NEAR(result.scale, fixture.expected_scale, fixture.tolerance_scale);
  EXPECT_NEAR(result.rms_normal_residual_degrees, fixture.expected_rms_normal_residual_degrees,
              kRmsNormalToleranceDegrees);
  EXPECT_NEAR(result.rms_offset_residual, fixture.expected_rms_offset_residual, kRmsOffsetTolerance);

  // Exactly the expected poses are marked unused.
  ASSERT_EQ(result.residuals.size(), fixture.correspondences.size());
  for (std::size_t k = 0; k < result.residuals.size(); ++k) {
    const bool expected_rejected =
        std::find(fixture.expected_rejected_pose_indices.begin(),
                  fixture.expected_rejected_pose_indices.end(),
                  static_cast<int>(k)) != fixture.expected_rejected_pose_indices.end();
    EXPECT_EQ(result.residuals[k].used, !expected_rejected) << "pose " << k;
  }
  EXPECT_EQ(result.poses_used,
            static_cast<int>(fixture.correspondences.size()) -
                static_cast<int>(fixture.expected_rejected_pose_indices.size()));
}

INSTANTIATE_TEST_SUITE_P(Fixtures, FixtureTest, ::testing::ValuesIn(tv::cases()),
                         [](const ::testing::TestParamInfo<tv::Case>& info) {
                           return info.param.name;
                         });

// ------------------------------------------------------------------------------------------
// ToString
// ------------------------------------------------------------------------------------------

TEST(ToStringTest, NamesEveryStatus) {
  EXPECT_STREQ(ToString(RegistrationStatus::Success), "Success");
  EXPECT_STREQ(ToString(RegistrationStatus::InvalidInput), "InvalidInput");
  EXPECT_STREQ(ToString(RegistrationStatus::TooFewPoses), "TooFewPoses");
  EXPECT_STREQ(ToString(RegistrationStatus::DegeneratePoses), "DegeneratePoses");
  EXPECT_STREQ(ToString(RegistrationStatus::ResidualsExceedThreshold), "ResidualsExceedThreshold");
}

// ------------------------------------------------------------------------------------------
// NormalizePlane
// ------------------------------------------------------------------------------------------

TEST(NormalizePlaneTest, ScalesToUnitNormal) {
  Plane plane;
  plane.normal = Eigen::Vector3d(kScaleFactor, 0.0, 0.0);
  plane.offset = kOffsetBeforeScaling;
  ASSERT_TRUE(NormalizePlane(plane, true));
  EXPECT_NEAR(plane.normal.norm(), 1.0, kExactTolerance);
  EXPECT_NEAR(plane.normal.x(), 1.0, kExactTolerance);
  EXPECT_NEAR(plane.offset, kOffsetAfterScaling, kExactTolerance);
}

TEST(NormalizePlaneTest, FlipsToPositiveOffset) {
  Plane plane;
  plane.normal = Eigen::Vector3d(0.0, kScaleFactor, 0.0);
  plane.offset = -kOffsetBeforeScaling;
  ASSERT_TRUE(NormalizePlane(plane, true));
  EXPECT_NEAR(plane.normal.y(), -1.0, kExactTolerance);  // the normal flipped with the offset
  EXPECT_NEAR(plane.offset, kOffsetAfterScaling, kExactTolerance);
}

TEST(NormalizePlaneTest, FlipsToNegativeOffset) {
  Plane plane;
  plane.normal = Eigen::Vector3d(0.0, 0.0, kScaleFactor);
  plane.offset = kOffsetBeforeScaling;
  ASSERT_TRUE(NormalizePlane(plane, false));
  EXPECT_NEAR(plane.normal.z(), -1.0, kExactTolerance);
  EXPECT_NEAR(plane.offset, -kOffsetAfterScaling, kExactTolerance);
}

TEST(NormalizePlaneTest, KeepsSignOfZeroOffset) {
  Plane plane;
  plane.normal = Eigen::Vector3d(0.0, kScaleFactor, 0.0);
  plane.offset = 0.0;
  ASSERT_TRUE(NormalizePlane(plane, true));
  EXPECT_NEAR(plane.normal.y(), 1.0, kExactTolerance);  // not flipped
  EXPECT_EQ(plane.offset, 0.0);
  ASSERT_TRUE(NormalizePlane(plane, false));
  EXPECT_NEAR(plane.normal.y(), 1.0, kExactTolerance);  // still not flipped
}

TEST(NormalizePlaneTest, RejectsZeroNormal) {
  Plane plane;
  plane.normal = Eigen::Vector3d::Zero();
  plane.offset = kOffsetBeforeScaling;
  EXPECT_FALSE(NormalizePlane(plane, true));
}

TEST(NormalizePlaneTest, RejectsNaN) {
  Plane plane;
  plane.normal = Eigen::Vector3d(1.0, std::numeric_limits<double>::quiet_NaN(), 0.0);
  plane.offset = kOffsetBeforeScaling;
  EXPECT_FALSE(NormalizePlane(plane, true));
}

// ------------------------------------------------------------------------------------------
// TransformPlane
// ------------------------------------------------------------------------------------------

TEST(TransformPlaneTest, MatchesInverseTransposeAndKeepsPointsOnThePlane) {
  std::mt19937 generator(kRandomSeed);
  std::uniform_real_distribution<double> translation(-kRandomTranslationBound,
                                                     kRandomTranslationBound);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = RandomRotation(generator);
  transform.translation() = Eigen::Vector3d(translation(generator), translation(generator),
                                            translation(generator));

  Plane plane;
  plane.normal = RandomUnitVector(generator);
  plane.offset = translation(generator);

  // Against the explicit 4x4 inverse transpose.
  const Eigen::Vector4d explicit_result =
      transform.matrix().inverse().transpose() * plane.Coefficients();
  const Plane transformed = TransformPlane(transform, plane);
  EXPECT_LE((transformed.Coefficients() - explicit_result).norm(), kExactTolerance);

  // A point on the original plane, mapped by T, lies on the transformed plane.
  const Eigen::Vector3d point_on_plane = -plane.offset * plane.normal;  // n . p + d = 0
  const Eigen::Vector3d mapped_point = transform * point_on_plane;
  EXPECT_NEAR(transformed.normal.dot(mapped_point) + transformed.offset, 0.0, kExactTolerance);
}

// ------------------------------------------------------------------------------------------
// SolveRotationBetweenDirections
// ------------------------------------------------------------------------------------------

TEST(SolveRotationTest, RecoversRotationFromThreeDirections) {
  std::mt19937 generator(kRandomSeed);
  const Eigen::Matrix3d truth = RandomRotation(generator);
  std::vector<Eigen::Vector3d> from;
  std::vector<Eigen::Vector3d> to;
  for (int k = 0; k < kThreeDirections; ++k) {
    from.push_back(RandomUnitVector(generator));
    to.push_back(truth * from.back());
  }
  EXPECT_LE((SolveRotationBetweenDirections(from, to) - truth).norm(), kExactTolerance);
}

TEST(SolveRotationTest, RecoversRotationFromTwoNonParallelDirections) {
  std::mt19937 generator(kRandomSeed);
  const Eigen::Matrix3d truth = RandomRotation(generator);
  std::vector<Eigen::Vector3d> from;
  std::vector<Eigen::Vector3d> to;
  for (int k = 0; k < kTwoDirections; ++k) {
    from.push_back(RandomUnitVector(generator));
    to.push_back(truth * from.back());
  }
  EXPECT_LE((SolveRotationBetweenDirections(from, to) - truth).norm(), kExactTolerance);
}

TEST(SolveRotationTest, ReturnsProperRotationForCoplanarDirections) {
  // All directions lie in the xy plane, so H has rank at most 2 and the third singular value is
  // zero; the determinant correction must still yield det = +1.  The targets are reflected
  // through that plane, which tempts the solver into returning a reflection.
  std::mt19937 generator(kRandomSeed);
  std::vector<Eigen::Vector3d> from;
  std::vector<Eigen::Vector3d> to;
  for (int k = 0; k < kCoplanarDirectionCount; ++k) {
    Eigen::Vector3d direction = RandomUnitVector(generator);
    direction.z() = 0.0;
    direction.normalize();
    from.push_back(direction);
    to.push_back(Eigen::Vector3d(direction.x(), -direction.y(), 0.0));
  }
  const Eigen::Matrix3d rotation = SolveRotationBetweenDirections(from, to);
  EXPECT_NEAR(rotation.determinant(), 1.0, kDeterminantTolerance);
  EXPECT_LE((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm(),
            kExactTolerance);
}

TEST(SolveRotationTest, HonorsWeights) {
  std::mt19937 generator(kRandomSeed);
  const Eigen::Matrix3d truth = RandomRotation(generator);
  std::vector<Eigen::Vector3d> from;
  std::vector<Eigen::Vector3d> to;
  for (int k = 0; k < kWeightedDirectionCount; ++k) {
    from.push_back(RandomUnitVector(generator));
    to.push_back(truth * from.back());
  }
  // Corrupt the last direction completely.
  to.back() = RandomUnitVector(generator);

  // Unweighted: the corrupted pair visibly spoils the solution.
  EXPECT_GT((SolveRotationBetweenDirections(from, to) - truth).norm(), kMinimumCorruptionError);

  // A zero weight on the corrupted pair removes its influence.
  std::vector<double> weights(kWeightedDirectionCount, 1.0);
  weights.back() = 0.0;
  EXPECT_LE((SolveRotationBetweenDirections(from, to, &weights) - truth).norm(), kExactTolerance);
}

// ------------------------------------------------------------------------------------------
// The linear oracle (spec Section 2.2)
// ------------------------------------------------------------------------------------------

TEST(OracleTest, MatchesInverseTransposeOnExactData) {
  const tv::Case& fixture = FixtureNamed("exact_thickness_17_poses");
  std::vector<Plane> sensor_planes;
  std::vector<Plane> world_planes;
  BuildOraclePlanes(fixture, sensor_planes, world_planes);

  const Eigen::Matrix4d xi = SolveLinearPlaneTransformOracle(sensor_planes, world_planes);
  const Eigen::Matrix4d expected =
      MatrixFromRowMajor(fixture.expected_sensor_to_world_row_major).inverse().transpose();
  EXPECT_LE((xi - expected).norm(), kOracleTolerance);

  const Eigen::Matrix3d block = xi.topLeftCorner<3, 3>();
  EXPECT_LE((block.transpose() * block - Eigen::Matrix3d::Identity()).norm(), kOracleTolerance);
}

TEST(OracleTest, IsUnconstrainedUnderNoiseButRepairableByPolarProjection) {
  const tv::Case& fixture = FixtureNamed("noisy_17_poses_matches_reference_solve");
  std::vector<Plane> sensor_planes;
  std::vector<Plane> world_planes;
  BuildOraclePlanes(fixture, sensor_planes, world_planes);

  const Eigen::Matrix4d xi = SolveLinearPlaneTransformOracle(sensor_planes, world_planes);
  const Eigen::Matrix3d block = xi.topLeftCorner<3, 3>();

  // Unconstrained: the block is not a rotation.
  const double orthogonality_defect =
      (block.transpose() * block - Eigen::Matrix3d::Identity()).norm();
  EXPECT_GT(orthogonality_defect, kNoisyOrthogonalityDefectMinimum);

  // Its nearest proper rotation (polar decomposition by SVD) agrees with the rigid solver.
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(block, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const double determinant_sign =
      (svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
  const Eigen::Matrix3d nearest_rotation =
      svd.matrixU() * Eigen::Vector3d(1.0, 1.0, determinant_sign).asDiagonal() *
      svd.matrixV().transpose();

  const RegistrationResult rigid =
      RegisterSensorToWorld(CorrespondencesOf(fixture), TargetOf(fixture), ParametersOf(fixture));
  ASSERT_EQ(rigid.status, RegistrationStatus::Success) << rigid.message;
  EXPECT_LE(RotationErrorDegrees(nearest_rotation, rigid.rotation),
            kPolarVersusRigidToleranceDegrees);
}

// ------------------------------------------------------------------------------------------
// Pose-count floor and invalid input
// ------------------------------------------------------------------------------------------

TEST(PoseCountTest, MinimumPoseCountCannotLowerTheSimilarityFloor) {
  const tv::Case& fixture = FixtureNamed("similarity_too_few_3_poses");
  RegistrationParameters parameters = ParametersOf(fixture);
  parameters.minimum_pose_count = static_cast<int>(fixture.correspondences.size());  // 3
  const RegistrationResult result =
      RegisterSensorToWorld(CorrespondencesOf(fixture), TargetOf(fixture), parameters);
  EXPECT_EQ(result.status, RegistrationStatus::TooFewPoses) << result.message;
}

TEST(PoseCountTest, MinimumPoseCountCanRaiseTheFloor) {
  const tv::Case& fixture = FixtureNamed("exact_minimal_3_poses");
  RegistrationParameters parameters = ParametersOf(fixture);
  parameters.minimum_pose_count = static_cast<int>(fixture.correspondences.size()) + 1;
  const RegistrationResult result =
      RegisterSensorToWorld(CorrespondencesOf(fixture), TargetOf(fixture), parameters);
  EXPECT_EQ(result.status, RegistrationStatus::TooFewPoses) << result.message;
}

// Three poses whose world plate normals are exactly e_x, e_y, e_z, with an identity
// sensor-to-world map: each sensor plane is the world plate plane itself.
std::vector<PlaneCorrespondence> OrthogonalNormalCorrespondences(const TargetPlane& target) {
  // Flange rotations that carry the flange +z axis (the target normal) onto each world axis.
  const std::vector<Eigen::Matrix3d> rotations = {
      Eigen::AngleAxisd(kQuarterTurnRadians, Eigen::Vector3d::UnitY()).toRotationMatrix(),   // z -> x
      Eigen::AngleAxisd(-kQuarterTurnRadians, Eigen::Vector3d::UnitX()).toRotationMatrix(),  // z -> y
      Eigen::Matrix3d::Identity()};                                                          // z -> z
  std::vector<PlaneCorrespondence> correspondences;
  for (const Eigen::Matrix3d& rotation : rotations) {
    PlaneCorrespondence correspondence;
    correspondence.flange_to_world.linear() = rotation;  // translation stays zero
    correspondence.sensor_plane = TransformPlane(correspondence.flange_to_world, target.plane);
    correspondences.push_back(correspondence);
  }
  return correspondences;
}

TEST(SpreadTest, OrthogonalNormalsHaveSpreadOneOverSqrtThree) {
  const TargetPlane target = TargetPlane::FromPlateThickness(kPositiveOffsetThickness);
  const RegistrationResult result = RegisterSensorToWorld(
      OrthogonalNormalCorrespondences(target), target, RegistrationParameters());
  ASSERT_EQ(result.status, RegistrationStatus::Success) << result.message;
  ASSERT_EQ(result.poses_used, kOrthogonalNormalCount);
  EXPECT_NEAR(result.normal_spread, 1.0 / std::sqrt(static_cast<double>(kOrthogonalNormalCount)),
              kSpreadTolerance);
  EXPECT_EQ(result.similarity_spread, 0.0);  // Rigid model: not computed
}

TEST(SpreadTest, ReportsSimilaritySpreadOnlyForTheSimilarityModel) {
  const tv::Case& fixture = FixtureNamed("exact_similarity_minimal_4_poses");
  const RegistrationResult similarity =
      RegisterSensorToWorld(CorrespondencesOf(fixture), TargetOf(fixture), ParametersOf(fixture));
  ASSERT_EQ(similarity.status, RegistrationStatus::Success) << similarity.message;
  EXPECT_GT(similarity.similarity_spread, 0.0);
  EXPECT_GT(similarity.normal_spread, 0.0);
}

TEST(InvalidInputTest, NaNInSensorPlaneNamesThePose) {
  const tv::Case& fixture = FixtureNamed("exact_thickness_17_poses");
  std::vector<PlaneCorrespondence> correspondences = CorrespondencesOf(fixture);
  correspondences[kInvalidPoseIndex].sensor_plane.normal.y() =
      std::numeric_limits<double>::quiet_NaN();
  const RegistrationResult result =
      RegisterSensorToWorld(correspondences, TargetOf(fixture), ParametersOf(fixture));
  EXPECT_EQ(result.status, RegistrationStatus::InvalidInput);
  EXPECT_NE(result.message.find(std::to_string(kInvalidPoseIndex)), std::string::npos)
      << result.message;
}

TEST(InvalidInputTest, ZeroSensorNormalIsRejected) {
  const tv::Case& fixture = FixtureNamed("exact_thickness_17_poses");
  std::vector<PlaneCorrespondence> correspondences = CorrespondencesOf(fixture);
  correspondences[kInvalidPoseIndex].sensor_plane.normal = Eigen::Vector3d::Zero();
  const RegistrationResult result =
      RegisterSensorToWorld(correspondences, TargetOf(fixture), ParametersOf(fixture));
  EXPECT_EQ(result.status, RegistrationStatus::InvalidInput);
  EXPECT_NE(result.message.find(std::to_string(kInvalidPoseIndex)), std::string::npos)
      << result.message;
}

TEST(InvalidInputTest, NonFiniteTargetIsRejected) {
  const tv::Case& fixture = FixtureNamed("exact_thickness_17_poses");
  const RegistrationResult result =
      RegisterSensorToWorld(CorrespondencesOf(fixture),
                            TargetPlane::FromPlateThickness(std::numeric_limits<double>::quiet_NaN()),
                            ParametersOf(fixture));
  EXPECT_EQ(result.status, RegistrationStatus::InvalidInput);
}

// ------------------------------------------------------------------------------------------
// Outlier rejection never drops below the pose floor
// ------------------------------------------------------------------------------------------

TEST(OutlierRejectionTest, StopsRatherThanDroppingBelowTheFloor) {
  // The fixture has one corrupted pose among its poses, and with its default parameters the
  // corrupted pose is dropped.  Raising minimum_pose_count to the full pose count forbids any
  // drop (dropping one pose would leave fewer than the floor), so the solve is kept as is, with
  // every pose used and the outlier inflating the residuals.
  const tv::Case& fixture = FixtureNamed("one_outlier_pose_rejected");
  std::vector<PlaneCorrespondence> correspondences = CorrespondencesOf(fixture);

  RegistrationParameters parameters = ParametersOf(fixture);
  parameters.outlier_rejection_rounds = kOutlierRoundsForOutlierCase;
  parameters.minimum_pose_count = static_cast<int>(correspondences.size());  // cannot drop any
  const RegistrationResult result =
      RegisterSensorToWorld(correspondences, TargetOf(fixture), parameters);
  EXPECT_EQ(result.poses_used, static_cast<int>(correspondences.size()));
  EXPECT_EQ(result.status, RegistrationStatus::ResidualsExceedThreshold);
}

}  // namespace
}  // namespace lri::registration
