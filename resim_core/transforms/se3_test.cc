#include "resim_core/transforms/se3.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "resim_core/transforms/liegroup_test_helpers.hh"

namespace resim::transforms {

// Test that an SE3 objects has the expected rotation and translation
// components.
// [param] se3 - the SE3 object to be tested.
// [param] rotation - SO3 representing the expected rotation.
// [param] translation - Vector representing the expected translation.
void check_se3_rotation_and_translation(
    const SE3 &se3,
    const SO3 &rotation,
    const Eigen::Vector3d &translation) {
  //
  EXPECT_TRUE(se3.rotation().is_approx(rotation));
  EXPECT_TRUE(se3.translation().isApprox(translation));
}

TEST(SE3ConstructorTest, RotationOnly) {
  for (const SO3 &a_from_b_rot : make_test_group_elements<SO3>()) {
    const SE3 a_from_b(a_from_b_rot);
    check_se3_rotation_and_translation(
        a_from_b,
        a_from_b_rot,
        Eigen::Vector3d::Zero());
  }
}

TEST(SE3ConstructorTest, TranslationOnly) {
  for (const Eigen::Vector3d &a_from_b_trans :
       make_test_vectors<Eigen::Vector3d>()) {
    const SE3 a_from_b(a_from_b_trans);
    check_se3_rotation_and_translation(
        a_from_b,
        SO3::identity(),
        a_from_b_trans);
  }
}

TEST(SE3ConstructorTest, RotationAndTranslation) {
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

TEST(SE3OperatorTest, ActionOnVector) {
  const SE3 orig_from_zero_one =
      SE3(SO3(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())),
          Eigen::Vector3d::UnitY());
  // Expected Location of axes vectors.
  const Eigen::Vector3d zero_one_x{0., 2., 0.};
  const Eigen::Vector3d zero_one_y{-1., 1., 0.};
  const Eigen::Vector3d zero_one_z{0., 1., 1.};
  EXPECT_TRUE(
      zero_one_x.isApprox(orig_from_zero_one * Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(
      zero_one_y.isApprox(orig_from_zero_one * Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(
      zero_one_z.isApprox(orig_from_zero_one * Eigen::Vector3d::UnitZ()));
}

TEST(SE3AdjointTest, AlgebraAdjointTimes) {}

TEST(SE3ArcLengthTest, IdentityArcLengthIsZero) {
  const SE3 a_from_a = SE3::identity();
  constexpr double ZERO = 0;
  EXPECT_EQ(a_from_a.arc_length(), ZERO);
}

TEST(SE3ArcLengthTest, UnitCircleArc) {
  const SE3 orig_from_one_zero = SE3(Eigen::Vector3d::UnitX());
  const SE3 orig_from_zero_one =
      SE3(SO3(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())),
          Eigen::Vector3d::UnitY());
  // Following transform is a quater arc of the unit circle.
  const SE3 zero_one_from_one_zero =
      orig_from_zero_one.inverse() * orig_from_one_zero;
  EXPECT_DOUBLE_EQ(zero_one_from_one_zero.arc_length(), M_PI_2);
}

TEST(SE3IsApproxTest, FloatingPointEquality) {
  const SE3 orig_from_zero_one =
      SE3(SO3(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())),
          Eigen::Vector3d::UnitY());
  // Same rotation and translation
  EXPECT_TRUE(orig_from_zero_one.is_approx(orig_from_zero_one));
  // Same rotation only
  const SE3 same_rot_only =
      SE3(SO3(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())),
          Eigen::Vector3d::UnitX());
  EXPECT_FALSE(orig_from_zero_one.is_approx(same_rot_only));
  // Same translation only
  const SE3 same_trans_only =
      SE3(SO3(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY())),
          Eigen::Vector3d::UnitY());
  EXPECT_FALSE(orig_from_zero_one.is_approx(same_trans_only));
  // Different rot and trans
  const SE3 different_rot_and_trans =
      SE3(SO3(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY())),
          Eigen::Vector3d::UnitX());
  EXPECT_FALSE(orig_from_zero_one.is_approx(different_rot_and_trans));
}

TEST(TangentVectorHelpers, RoundTripConsistent) {
  const SO3::TangentVector alg_rot = SO3::TangentVector::Ones();
  const Eigen::Vector3d alg_trans = Eigen::Vector3d::Ones() * 2.;
  SE3::TangentVector alg = SE3::tangent_vector_from_parts(alg_rot, alg_trans);
  EXPECT_TRUE(alg_rot.isApprox(SE3::tangent_vector_rotation_part(alg)));
  EXPECT_TRUE(alg_trans.isApprox(SE3::tangent_vector_translation_part(alg)));
}

// This test verifies that we can set parts of the TangentVector using
// the rotation and translation part helpers.
TEST(TangentVectorHelpers, VectorPartMutation) {
  // SETUP
  SE3::TangentVector alg{SE3::TangentVector::Zero()};

  const SO3::TangentVector new_alg_rot = SO3::TangentVector::Ones();
  const Eigen::Vector3d new_alg_trans = Eigen::Vector3d::Ones() * 2.;

  // ACTION
  SE3::tangent_vector_rotation_part(alg) = new_alg_rot;
  SE3::tangent_vector_translation_part(alg) = new_alg_trans;

  // VERIFICATION
  const SE3::TangentVector expected_alg{
      SE3::tangent_vector_from_parts(new_alg_rot, new_alg_trans)};
  EXPECT_EQ(alg, expected_alg);
}

}  // namespace resim::transforms
