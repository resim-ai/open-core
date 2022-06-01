#include "transforms/se3.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "transforms/liegroup_test_helpers.hh"

namespace resim {
namespace transforms {

// Test that an SE3 objects has the expected rotation and translation
// components.
// [param] se3 - the SE3 object to be tested.
// [param] rotation - SO3 representing the expected rotation.
// [param] translation - Vector representing the expected translation.
void check_se3_rotation_and_translation(
    const SE3 &se3,
    const SO3 &rotation,
    const Eigen::Vector3d translation) {
  //
  EXPECT_TRUE(se3.rotation().is_approx(rotation));
  EXPECT_TRUE(se3.translation().isApprox(translation));
}

TEST(SE3ConstructorTest, rotation_only) {
  for (const SO3 &a_from_b_rot : make_test_group_elements<SO3>()) {
    const SE3 a_from_b(a_from_b_rot);
    check_se3_rotation_and_translation(
        a_from_b,
        a_from_b_rot,
        Eigen::Vector3d::Zero());
  }
}

TEST(SE3ConstructorTest, translation_only) {
  for (const Eigen::Vector3d &a_from_b_trans :
       make_test_vectors<Eigen::Vector3d>()) {
    const SE3 a_from_b(a_from_b_trans);
    check_se3_rotation_and_translation(
        a_from_b,
        SO3::identity(),
        a_from_b_trans);
  }
}

TEST(SE3ConstructorTest, rotation_and_translation) {
  const auto trans_elements = make_test_vectors<Eigen::Vector3d>();
  const auto rot_elements = make_test_group_elements<SO3>();
  const unsigned int element_count = trans_elements.size();
  EXPECT_EQ(element_count, rot_elements.size());
  for (int i = 0; i < element_count; ++i) {
    const SE3 a_from_b(rot_elements.at(i), trans_elements.at(i));
    check_se3_rotation_and_translation(
        a_from_b,
        rot_elements.at(i),
        trans_elements.at(i));
  }
}

TEST(SE3ArcLengthTest, identity_arc_length_is_zero) {
  const SE3 a_from_a = SE3::identity();
  constexpr double ZERO = 0;
  EXPECT_EQ(a_from_a.arc_length(), ZERO);
}

TEST(SE3ArcLengthTest, unit_circle_arc) {
  constexpr double HALF_PI = M_PI / 2;
  const SE3 orig_from_one_zero = SE3(Eigen::Vector3d::UnitX());
  const SE3 orig_from_zero_one =
      SE3(SO3(Eigen::AngleAxisd(HALF_PI, Eigen::Vector3d::UnitZ())),
          Eigen::Vector3d::UnitY());
  // Following transform is a quater arc of the unit circle.
  const SE3 zero_one_from_one_zero =
      orig_from_zero_one.inverse() * orig_from_one_zero;
  EXPECT_DOUBLE_EQ(zero_one_from_one_zero.arc_length(), HALF_PI);
}

TEST(TangentVectorHelpers, round_trip_consistent) {
  const SO3::TangentVector alg_rot = SO3::TangentVector::Ones();
  const Eigen::Vector3d alg_trans = Eigen::Vector3d::Ones() * 2.;
  SE3::TangentVector alg = SE3::tangent_vector_from_parts(alg_rot, alg_trans);
  EXPECT_TRUE(alg_rot.isApprox(SE3::tangent_vector_rotation_part(alg)));
  EXPECT_TRUE(alg_trans.isApprox(SE3::tangent_vector_translation_part(alg)));
}

}  // namespace transforms
}  // namespace resim
