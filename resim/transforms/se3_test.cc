#include "resim/transforms/se3.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "resim/transforms/liegroup_test_helpers.hh"
#include "resim/transforms/so3.hh"

namespace resim::transforms {

namespace {
// Create some test frames.
constexpr unsigned int DIMS = 3;
const Frame<DIMS> A = Frame<DIMS>::new_frame();
const Frame<DIMS> B = Frame<DIMS>::new_frame();
const Frame<DIMS> C = Frame<DIMS>::new_frame();

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
}  // namespace

TEST(SE3ConstructorTest, RotationOnly) {
  for (const SO3 &a_from_b_rot : make_test_group_elements<SO3>()) {
    const SE3 a_from_b(a_from_b_rot);
    check_se3_rotation_and_translation(
        a_from_b,
        a_from_b_rot,
        Eigen::Vector3d::Zero());
  }
}

TEST(SE3FramedConstructorTest, RotationOnly) {
  for (const SO3 &a_from_b_rot : make_test_group_elements<SO3>()) {
    const SE3 a_from_b(a_from_b_rot, A, B);
    check_se3_rotation_and_translation(
        a_from_b,
        a_from_b_rot,
        Eigen::Vector3d::Zero());
    EXPECT_TRUE(a_from_b.verify_frames(A, B));
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

TEST(SE3FramedConstructorTest, TranslationOnly) {
  for (const Eigen::Vector3d &a_from_b_trans :
       make_test_vectors<Eigen::Vector3d>()) {
    const SE3 a_from_b(a_from_b_trans, A, B);
    check_se3_rotation_and_translation(
        a_from_b,
        SO3::identity(),
        a_from_b_trans);
    EXPECT_TRUE(a_from_b.verify_frames(A, B));
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

TEST(SE3FramedConstructorTest, RotationAndTranslation) {
  const auto trans_elements = make_test_vectors<Eigen::Vector3d>();
  const auto rot_elements = make_test_group_elements<SO3>();
  const unsigned int element_count = trans_elements.size();
  EXPECT_EQ(element_count, rot_elements.size());
  for (int i = 0; i < element_count; ++i) {
    const SE3 a_from_b(rot_elements.at(i), trans_elements.at(i), A, B);
    check_se3_rotation_and_translation(
        a_from_b,
        rot_elements.at(i),
        trans_elements.at(i));
    EXPECT_TRUE(a_from_b.verify_frames(A, B));
  }
}

TEST(SE3FramedConstructorTest, SO3FramesAreStripped) {
  const SO3 framed_rotation = SO3::identity(A, B);

  // Try setting the framed rotation in relevant SE3 constructors, then test
  // that the frames are stripped.

  const SE3 test_se3_1(framed_rotation);
  EXPECT_FALSE(test_se3_1.rotation().is_framed());

  const SE3 test_se3_2(framed_rotation, A, B);
  EXPECT_TRUE(test_se3_2.is_framed());
  EXPECT_FALSE(test_se3_2.rotation().is_framed());

  const SE3 test_se3_3(Eigen::Vector3d::Ones(), A, B);
  EXPECT_TRUE(test_se3_3.is_framed());
  EXPECT_FALSE(test_se3_3.rotation().is_framed());

  const SE3 test_se3_4(framed_rotation, Eigen::Vector3d::Ones(), A, B);
  EXPECT_TRUE(test_se3_4.is_framed());
  EXPECT_FALSE(test_se3_4.rotation().is_framed());
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

TEST(SE3OperatorTest, RotateVector) {
  const SE3 orig_from_zero_one =
      SE3(SO3(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())),
          Eigen::Vector3d::UnitY());
  EXPECT_TRUE(
      orig_from_zero_one.rotate(Eigen::Vector3d::UnitX())
          .isApprox(orig_from_zero_one.rotation() * Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(
      orig_from_zero_one.rotate(Eigen::Vector3d::UnitY())
          .isApprox(orig_from_zero_one.rotation() * Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(
      orig_from_zero_one.rotate(Eigen::Vector3d::UnitZ())
          .isApprox(orig_from_zero_one.rotation() * Eigen::Vector3d::UnitZ()));
}

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

TEST(DistanceTest, SE3DistanceSucceeds) {
  constexpr double TWO = 2.0;

  SE3 a_from_c = SE3::exp(SE3::TangentVector::Ones(), A, C);
  SE3 b_from_c = SE3::exp(TWO * SE3::TangentVector::Ones(), B, C);

  const double d = se3_distance(a_from_c, b_from_c);

  EXPECT_NEAR(d, (b_from_c * a_from_c.inverse()).translation().norm(), 1e-7);
}

TEST(DistanceTest, SE3DistanceWrongFrame) {
  SE3 a_from_c = SE3::exp(SE3::TangentVector::Ones(), A, C);
  SE3 a_from_b = SE3::exp(SE3::TangentVector::Ones(), A, B);

  EXPECT_THROW(se3_distance(a_from_c, a_from_b), AssertException);
}

TEST(DistanceTest, SE3InverseDistanceSucceeds) {
  constexpr double TWO = 2.0;

  SE3 c_from_a = SE3::exp(SE3::TangentVector::Ones(), C, A);
  SE3 c_from_b = SE3::exp(TWO * SE3::TangentVector::Ones(), C, B);

  const double d = se3_inverse_distance(c_from_a, c_from_b);

  EXPECT_NEAR(d, (c_from_a.inverse() * c_from_b).translation().norm(), 1e-7);
}

TEST(DistanceTest, SE3InverseDistanceWrongFrame) {
  SE3 c_from_a = SE3::exp(SE3::TangentVector::Ones(), C, A);
  SE3 b_from_a = SE3::exp(SE3::TangentVector::Ones(), B, A);

  EXPECT_THROW(se3_inverse_distance(c_from_a, b_from_a), AssertException);
}

}  // namespace resim::transforms
