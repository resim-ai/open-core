#include "resim_core/transforms/so3.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>
#include <vector>

#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/utils/random_vector.hh"

namespace resim::transforms {

namespace {

// A simple helper to generate random quaternions. Note that these are *NOT*
// uniformly distributed orientations.
// TODO(https://app.asana.com/0/1202178773526279/1203262688903982/f)
template <typename Rng>
Eigen::Quaterniond random_quaternion(Rng &&rng) {
  return Eigen::Quaterniond{
      testing::random_vector<Eigen::Vector4d>(rng).normalized()};
}

}  // namespace

// Please Note: Here are the specialized tests for the SO3 class. The SO3 class
// is also covered by a number of general Liegrooup tests that can be found in
// liegroup_test.cc

TEST(SO3ConstructorTest, AngleAxisZero) {
  // Initialize SO3 with a zero angle angle_axis argument.
  constexpr double ZERO_ANGLE = 0;
  const SO3 no_rotation(
      Eigen::AngleAxisd(ZERO_ANGLE, Eigen::Vector3d::UnitX()));
  // Verify that the rotation matrix is identity.
  EXPECT_TRUE(
      no_rotation.rotation_matrix().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(SO3ConstructorTest, AngleAxisArbitrary) {
  // Initialize SO3 with a arbitrary angle angle_axis argument.
  constexpr double ANGLE = 0.732;
  const Eigen::AngleAxisd rotation_aa(ANGLE, Eigen::Vector3d::UnitY());
  const SO3 rotation_so3(rotation_aa);
  // Verify that rotation matrices of axis angle and so3 agree.
  EXPECT_TRUE(
      rotation_so3.rotation_matrix().isApprox(rotation_aa.toRotationMatrix()));
}

TEST(SO3ConstructorTest, MatrixIdentity) {
  // Initialize SO3 with an identity matrix.
  const SO3 from_id_mat(Eigen::Matrix3d::Identity());
  // Initializes to identity
  EXPECT_TRUE(
      from_id_mat.rotation_matrix().isApprox(Eigen::Matrix3d::Identity()));
  // Compare to identity SO3
  const SO3 identity = SO3::identity();
  EXPECT_TRUE(identity.is_approx(from_id_mat));
}

TEST(SO3ConstructorTest, quaternion) {
  constexpr int TRIES = 7;
  constexpr unsigned int SEED = 42;
  // Make a deterministic seed.
  std::mt19937 rng{SEED};
  // Build a vector of random Quaternions.
  std::vector<Eigen::Quaterniond> quats;
  quats.reserve(TRIES);
  for (int i = 0; i < TRIES; ++i) {
    quats.push_back(random_quaternion(rng));
  }
  // Create an arbitrary three vector
  const Eigen::Vector3d three_vec{testing::random_vector<Eigen::Vector3d>(rng)};
  for (const Eigen::Quaterniond &a_from_b_quat : quats) {
    // Build an SO3 from the quaternion
    const SO3 a_from_b_so3(a_from_b_quat);
    // Apply the action of the Quaternion to the test vector.
    const Eigen::Vector3d vec_quat = a_from_b_quat * three_vec;
    // Apply the action of the SO3 to the test vector.
    const Eigen::Vector3d vec_so3 = a_from_b_so3 * three_vec;
    // Action should be identical
    EXPECT_TRUE(vec_quat.isApprox(vec_so3));
  }
}

TEST(SO3OperatorTest, ActionOnVector) {
  const SO3 orig_from_half_pi =
      SO3(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  // Expected Location of axes vectors.
  const Eigen::Vector3d zero_one_x{0., 1., 0.};
  const Eigen::Vector3d zero_one_y{-1., 0., 0.};
  const Eigen::Vector3d zero_one_z{0., 0., 1.};
  EXPECT_TRUE(
      zero_one_x.isApprox(orig_from_half_pi * Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(
      zero_one_y.isApprox(orig_from_half_pi * Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(
      zero_one_z.isApprox(orig_from_half_pi * Eigen::Vector3d::UnitZ()));
}

TEST(SO3Inverse, CompareToMatrixInverse) {
  const auto test_so3s = make_test_group_elements<SO3>();
  for (const SO3 &a_from_b : test_so3s) {
    const Eigen::Matrix3d &a_from_b_mat = a_from_b.rotation_matrix();
    const SO3 b_from_a = SO3(a_from_b_mat.inverse());
    EXPECT_TRUE(a_from_b.inverse().is_approx(b_from_a));
  }
}

TEST(SO3Interp, InterpHalfway) {
  constexpr double ANGLE = M_PI / 4.;
  constexpr double HALFWAY = 0.5;
  for (const double angle : {ANGLE, -ANGLE}) {
    const SO3 a_from_b(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
    // Create a renamed reference to make explicit that the rotation
    // is to be applied twice sequentially.
    const SO3 &b_from_c = a_from_b;
    const SO3 a_from_c = a_from_b * b_from_c;
    EXPECT_TRUE(a_from_c.interp(HALFWAY).is_approx(a_from_b));
  }
}

TEST(SO3Interp, ExtrapDouble) {
  constexpr double ANGLE = M_PI / 4.;
  constexpr double DOUBLE = 2.;
  for (const double angle : {ANGLE, -ANGLE}) {
    const SO3 a_from_b(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
    // Create a renamed reference to make explicit that the rotation
    // is to be applied twice sequentially.
    const SO3 &b_from_c = a_from_b;
    const SO3 a_from_c = a_from_b * b_from_c;
    EXPECT_TRUE(a_from_b.interp(DOUBLE).is_approx(a_from_c));
  }
}

}  // namespace resim::transforms
