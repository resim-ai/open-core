// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/transforms/so3.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>
#include <vector>

#include "resim/math/is_approx.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/liegroup_test_helpers.hh"

namespace resim::transforms {

// Please Note: Here are the specialized tests for the SO3 class. The SO3 class
// is also covered by a number of general Lie group tests that can be found in
// liegroup_test.cc

namespace {
// Create some test frames.
constexpr unsigned int DIMS = 3;
const Frame<DIMS> A = Frame<DIMS>::new_frame();
const Frame<DIMS> B = Frame<DIMS>::new_frame();
}  // namespace

TEST(SO3ConstructorTest, AngleAxisZero) {
  // Initialize SO3 with a zero angle angle_axis argument.
  constexpr double ZERO_ANGLE = 0;
  const SO3 no_rotation(
      Eigen::AngleAxisd(ZERO_ANGLE, Eigen::Vector3d::UnitX()));
  // Verify that the rotation matrix is identity.
  EXPECT_TRUE(math::is_approx(
      no_rotation.rotation_matrix(),
      Eigen::Matrix3d::Identity()));
}

TEST(SO3FramedConstructorTest, AngleAxisZero) {
  // Initialize SO3 with a zero angle angle_axis argument.
  constexpr double ZERO_ANGLE = 0;
  const SO3 no_rotation(
      Eigen::AngleAxisd(ZERO_ANGLE, Eigen::Vector3d::UnitX()),
      A,
      B);
  // Verify that the rotation matrix is identity.
  EXPECT_TRUE(math::is_approx(
      no_rotation.rotation_matrix(),
      Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(no_rotation.verify_frames(A, B));
}

TEST(SO3ConstructorTest, AngleAxisArbitrary) {
  // Initialize SO3 with a arbitrary angle angle_axis argument.
  constexpr double ANGLE = 0.732;
  const Eigen::AngleAxisd rotation_aa(ANGLE, Eigen::Vector3d::UnitY());
  const SO3 rotation_so3(rotation_aa);
  // Verify that rotation matrices of axis angle and so3 agree.
  EXPECT_TRUE(math::is_approx(
      rotation_so3.rotation_matrix(),
      rotation_aa.toRotationMatrix()));
}

TEST(SO3FramedConstructorTest, AngleAxisArbitrary) {
  // Initialize SO3 with a arbitrary angle angle_axis argument.
  constexpr double ANGLE = 0.732;
  const Eigen::AngleAxisd rotation_aa(ANGLE, Eigen::Vector3d::UnitY());
  const SO3 rotation_so3(rotation_aa, A, B);
  // Verify that rotation matrices of axis angle and so3 agree.
  EXPECT_TRUE(math::is_approx(
      rotation_so3.rotation_matrix(),
      rotation_aa.toRotationMatrix()));
  EXPECT_TRUE(rotation_so3.verify_frames(A, B));
}

TEST(SO3ConstructorTest, AngleWithAxisZero) {
  // Initialize SO3 with a zero angle and unit x axis argument.
  constexpr double ZERO_ANGLE = 0.;
  const SO3 no_rotation(ZERO_ANGLE, {1., 0., 0.});
  // Verify that the rotation matrix is identity.
  EXPECT_TRUE(math::is_approx(
      no_rotation.rotation_matrix(),
      Eigen::Matrix3d::Identity()));
}

TEST(SO3FramedConstructorTest, AngleWithAxisZero) {
  // Initialize SO3 with a zero angle and unit x axis argument.
  constexpr double ZERO_ANGLE = 0.;
  const SO3 no_rotation(ZERO_ANGLE, {1., 0., 0.}, A, B);
  // Verify that the rotation matrix is identity.
  EXPECT_TRUE(math::is_approx(
      no_rotation.rotation_matrix(),
      Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(no_rotation.verify_frames(A, B));
}

TEST(SO3ConstructorTest, AngleWithAxisArbitrary) {
  // Initialize SO3 with a arbitrary angle angle and unit y axis argument.
  constexpr double ANGLE = 0.732;
  const SO3 rotation_so3(ANGLE, {0., 1., 0.});
  const Eigen::AngleAxisd rotation_aa(ANGLE, Eigen::Vector3d::UnitY());
  // Verify that rotation matrices of axis angle and so3 agree.
  EXPECT_TRUE(math::is_approx(
      rotation_so3.rotation_matrix(),
      rotation_aa.toRotationMatrix()));
}

TEST(SO3ConstructorTest, MatrixIdentity) {
  // Initialize SO3 with an identity matrix.
  const SO3 from_id_mat(Eigen::Matrix3d::Identity());
  // Initializes to identity
  EXPECT_TRUE(math::is_approx(
      from_id_mat.rotation_matrix(),
      Eigen::Matrix3d::Identity()));
  // Compare to identity SO3
  const SO3 identity = SO3::identity();
  EXPECT_TRUE(identity.is_approx(from_id_mat));
}

TEST(SO3FramedConstructorTest, MatrixIdentity) {
  // Initialize SO3 with an identity matrix.
  const SO3 from_id_mat(Eigen::Matrix3d::Identity(), A, B);
  // Initializes to identity
  EXPECT_TRUE(math::is_approx(
      from_id_mat.rotation_matrix(),
      Eigen::Matrix3d::Identity()));
  // Compare to identity SO3
  const SO3 identity = SO3::identity(A, B);
  EXPECT_TRUE(identity.is_approx(from_id_mat));
  EXPECT_TRUE(from_id_mat.verify_frames(A, B));
}

TEST(SO3ConstructorTest, FromQuaternion) {
  constexpr int TRIES = 7;
  constexpr unsigned int SEED = 42;
  // Make a deterministic seed.
  std::mt19937 rng{SEED};
  // Build a vector of random Quaternions.
  std::vector<Eigen::Quaterniond> quats;
  quats.reserve(TRIES);
  for (int i = 0; i < TRIES; ++i) {
    quats.push_back(testing::random_quaternion(rng));
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
    EXPECT_TRUE(math::is_approx(vec_quat, vec_so3));
  }
}

TEST(SO3FramedConstructorTest, quaternion) {
  constexpr int TRIES = 7;
  constexpr unsigned int SEED = 42;
  // Make a deterministic seed.
  std::mt19937 rng{SEED};
  // Build a vector of random Quaternions.
  std::vector<Eigen::Quaterniond> quats;
  quats.reserve(TRIES);
  for (int i = 0; i < TRIES; ++i) {
    quats.push_back(testing::random_quaternion(rng));
  }
  // Create an arbitrary three vector
  const Eigen::Vector3d three_vec{testing::random_vector<Eigen::Vector3d>(rng)};
  for (const Eigen::Quaterniond &a_from_b_quat : quats) {
    // Build an SO3 from the quaternion
    const SO3 a_from_b_so3(a_from_b_quat, A, B);
    // Apply the action of the Quaternion to the test vector.
    const Eigen::Vector3d vec_quat = a_from_b_quat * three_vec;
    // Apply the action of the SO3 to the test vector.
    const Eigen::Vector3d vec_so3 = a_from_b_so3 * three_vec;
    // Action should be identical
    EXPECT_TRUE(math::is_approx(vec_quat, vec_so3));
    EXPECT_TRUE(a_from_b_so3.verify_frames(A, B));
  }
}

TEST(SO3OperatorTest, ActionOnVector) {
  const SO3 orig_from_half_pi =
      SO3(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  // Expected Location of axes vectors.
  const Eigen::Vector3d zero_one_x{0., 1., 0.};
  const Eigen::Vector3d zero_one_y{-1., 0., 0.};
  const Eigen::Vector3d zero_one_z{0., 0., 1.};
  EXPECT_TRUE(math::is_approx(
      zero_one_x,
      orig_from_half_pi * Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(math::is_approx(
      zero_one_y,
      orig_from_half_pi * Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(math::is_approx(
      zero_one_z,
      orig_from_half_pi * Eigen::Vector3d::UnitZ()));
}

TEST(SO3OperatorTest, RotateVector) {
  const SO3 orig_from_half_pi =
      SO3(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(math::is_approx(
      orig_from_half_pi.rotate(Eigen::Vector3d::UnitX()),
      orig_from_half_pi * Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(math::is_approx(
      orig_from_half_pi.rotate(Eigen::Vector3d::UnitY()),
      orig_from_half_pi * Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(math::is_approx(
      orig_from_half_pi.rotate(Eigen::Vector3d::UnitZ()),
      orig_from_half_pi * Eigen::Vector3d::UnitZ()));
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

TEST(SO3Quaterniond, TestGetQuaternion) {
  // SETUP
  constexpr unsigned SEED = 594U;
  std::mt19937 rng_{SEED};

  constexpr int NUM_TESTS = 1000;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Eigen::Quaterniond test_quaternion{testing::random_quaternion(rng_)};

    // ACTION
    const Eigen::Quaterniond result_quaternion{
        SO3{test_quaternion}.quaternion()};

    // VERIFICATION
    // Both +q and -q represent the same orientation for any quaternion q:
    const Eigen::Quaterniond minus_one{-1., 0., 0., 0.};
    EXPECT_TRUE(
        math::is_approx(result_quaternion.vec(), test_quaternion.vec()) or
        math::is_approx(
            result_quaternion.vec(),
            (minus_one * test_quaternion).vec()));
  }
}

// Test that the differential of the exponential is correct using finite
// differences.
TEST(SO3Test, TestExpDiff) {
  for (const SO3::TangentVector &vec : make_test_algebra_elements<SO3>()) {
    constexpr double EPS = 1e-9;
    const SO3 unperturbed{SO3::exp(vec)};

    const SO3::TangentMapping d_exp{SO3::exp_diff(vec)};
    for (int ii = 0; ii < SO3::DOF; ++ii) {
      const SO3::TangentVector delta{EPS * SO3::TangentVector::Unit(ii)};
      const SO3 perturbed{SO3::exp(vec + delta)};
      const SO3::TangentVector expected_derivative{
          (perturbed * unperturbed.inverse()).log() / EPS};

      constexpr double TOLERANCE = 1e-6;
      EXPECT_TRUE(math::is_approx(
          expected_derivative,
          d_exp * SO3::TangentVector::Unit(ii),
          TOLERANCE));
    }
  }
}

}  // namespace resim::transforms
