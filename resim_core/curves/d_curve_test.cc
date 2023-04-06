#include "resim_core/curves/d_curve.hh"

#include <gtest/gtest.h>

#include "resim_core/assert/assert.hh"
#include "resim_core/curves/d_curve_test_helpers.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::curves {

namespace {
using SE3 = transforms::SE3;
using SO3 = transforms::SO3;
using FSE3 = transforms::FSE3;
}  // namespace

template <typename T>
class DCurveTests : public ::testing::Test {
 public:
  inline static const auto REF_FRAME = transforms::Frame<T::DIMS>::new_frame();
  inline static const auto PNT_FRAME = transforms::Frame<T::DIMS>::new_frame();
  static T point_to_append();
  double rotation_angle_rad(const T &ref_from_point) const;
};

template <>
SE3 DCurveTests<SE3>::point_to_append() {
  SE3 ref_from_000deg(Eigen::Vector3d::UnitX());
  return ref_from_000deg;
}

template <>
FSE3 DCurveTests<FSE3>::point_to_append() {
  const SE3 ref_from_000deg = DCurveTests<SE3>::point_to_append();
  return FSE3(ref_from_000deg, REF_FRAME, PNT_FRAME);
}

template <>
double DCurveTests<SE3>::rotation_angle_rad(const SE3 &ref_from_point) const {
  return ref_from_point.rotation().log().norm();
}

template <>
double DCurveTests<FSE3>::rotation_angle_rad(const FSE3 &ref_from_point) const {
  return ref_from_point.rotation().log().norm();
}

using LieGroupTypes = ::testing::Types<FSE3, SE3>;
TYPED_TEST_SUITE(DCurveTests, LieGroupTypes);

TYPED_TEST(DCurveTests, EmptyCurveConstruction) {
  DCurve<TypeParam> curve_a;
  EXPECT_TRUE(curve_a.control_pts().empty());
}

TYPED_TEST(DCurveTests, ConstructionFromPoints) {
  const std::vector<TypeParam> points = DCurveCircle<TypeParam>::points(
      DCurveTests<TypeParam>::REF_FRAME,
      DCurveTests<TypeParam>::PNT_FRAME);
  const DCurve<TypeParam> curve_a(points);
  EXPECT_EQ(curve_a.control_pts().size(), points.size());

  // Curve is 0-270 degrees of the unit circle so length should be 3pi/2.
  constexpr double EXPECTED_LENGTH_M = 3. * M_PI / 2.;
  EXPECT_DOUBLE_EQ(curve_a.curve_length(), EXPECTED_LENGTH_M);
}

TYPED_TEST(DCurveTests, CopyAndAssignment) {
  const std::vector<TypeParam> points = DCurveCircle<TypeParam>::points(
      DCurveTests<TypeParam>::REF_FRAME,
      DCurveTests<TypeParam>::PNT_FRAME);
  const DCurve<TypeParam> curve_a(points);
  // NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
  const DCurve<TypeParam> curve_b(curve_a);
  // NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
  const DCurve<TypeParam> curve_c = curve_a;
  EXPECT_DOUBLE_EQ(curve_a.curve_length(), curve_b.curve_length());
  EXPECT_DOUBLE_EQ(curve_a.curve_length(), curve_c.curve_length());
}

TYPED_TEST(DCurveTests, ListInitializationConstruction) {
  constexpr unsigned int NUM_POINTS = 3;
  const std::vector<TypeParam> points = DCurveCircle<TypeParam>::points(
      DCurveTests<TypeParam>::REF_FRAME,
      DCurveTests<TypeParam>::PNT_FRAME);
  ASSERT_GE(points.size(), NUM_POINTS);
  DCurve<TypeParam> curve{points.at(0), points.at(1), points.at(2)};
  // Confirm the control points are the same.
  for (unsigned int i = 0; i < NUM_POINTS; ++i) {
    EXPECT_TRUE(
        curve.control_pts().at(i).ref_from_control->is_approx(points.at(i)));
  }
}

TYPED_TEST(DCurveTests, ListInitializationAssignment) {
  constexpr unsigned int NUM_POINTS = 3;
  const std::vector<TypeParam> points = DCurveCircle<TypeParam>::points(
      DCurveTests<TypeParam>::REF_FRAME,
      DCurveTests<TypeParam>::PNT_FRAME);
  ASSERT_GE(points.size(), NUM_POINTS);
  DCurve<TypeParam> curve = {points.at(0), points.at(1), points.at(2)};
  // Confirm the control points are the same.
  for (unsigned int i = 0; i < NUM_POINTS; ++i) {
    EXPECT_TRUE(
        curve.control_pts().at(i).ref_from_control->is_approx(points.at(i)));
  }
}

TYPED_TEST(DCurveTests, ListInitializationAppend) {
  constexpr unsigned int NUM_POINTS = 3;
  const std::vector<TypeParam> points = DCurveCircle<TypeParam>::points(
      DCurveTests<TypeParam>::REF_FRAME,
      DCurveTests<TypeParam>::PNT_FRAME);
  ASSERT_GE(points.size(), NUM_POINTS);
  DCurve<TypeParam> curve = {points.at(0)};
  curve.append({points.at(1), points.at(2)});
  // Confirm the control points are the same.
  for (unsigned int i = 0; i < NUM_POINTS; ++i) {
    EXPECT_TRUE(
        curve.control_pts().at(i).ref_from_control->is_approx(points.at(i)));
  }
}

TYPED_TEST(DCurveTests, Append) {
  DCurve<TypeParam> curve_a(DCurveCircle<TypeParam>::points(
      DCurveTests<TypeParam>::REF_FRAME,
      DCurveTests<TypeParam>::PNT_FRAME));
  // Complete the circle:
  curve_a.append(this->point_to_append());
  // Confirm data member size.
  constexpr unsigned int EXPECTED_SIZE = 5;
  EXPECT_EQ(curve_a.control_pts().size(), EXPECTED_SIZE);
  EXPECT_EQ(curve_a.segments().size(), EXPECTED_SIZE - 1);
  // Confim curve lengh.
  // Circumference of unit circle.
  constexpr double EXPECTED_LENGTH_M = 2. * M_PI;
  EXPECT_DOUBLE_EQ(curve_a.curve_length(), EXPECTED_LENGTH_M);
}

TYPED_TEST(DCurveTests, DataMembersSanityChecks) {
  const std::vector<TypeParam> points = DCurveCircle<TypeParam>::points(
      DCurveTests<TypeParam>::REF_FRAME,
      DCurveTests<TypeParam>::PNT_FRAME);
  const DCurve<TypeParam> curve_a(points);
  for (unsigned int i = 0; i < points.size(); ++i) {
    EXPECT_TRUE(
        points[i].is_approx(*curve_a.control_pts()[i].ref_from_control));
    if (i > 0) {
      EXPECT_TRUE(
          points[i - 1].is_approx(*curve_a.segments()[i - 1].ref_from_orig));
      EXPECT_TRUE(
          points[i].is_approx(*curve_a.segments()[i - 1].ref_from_dest));
    }
  }
}

TYPED_TEST(DCurveTests, QueryPoints) {
  // Because curve segments are geodesic curves an accurate circle can be
  // created from just a few points. We can use this fact to help verify the
  // correctness of our implementation. All points sampled from our test curve
  // should be on the unit circle.
  DCurve<TypeParam> curve_a(DCurveCircle<TypeParam>::points(
      DCurveTests<TypeParam>::REF_FRAME,
      DCurveTests<TypeParam>::PNT_FRAME));
  // Complete the circle:
  curve_a.append(this->point_to_append());
  // Sample the curve at every degree on the circle.
  constexpr unsigned int CIRCLE_DEGS = 360;
  const double increment =
      curve_a.curve_length() / static_cast<double>(CIRCLE_DEGS);
  for (unsigned int i = 0; i < CIRCLE_DEGS; ++i) {
    const double sample_arc_length = i * increment;
    TypeParam ref_from_point = curve_a.point_at(sample_arc_length);
    // Negative Unit X should always be at the [0, 0]
    const auto expected_0 = ref_from_point * -Eigen::Vector3d::UnitX();
    EXPECT_TRUE(expected_0.isZero());
    const double angle_rad = this->rotation_angle_rad(ref_from_point);
    constexpr double PI_DEGS = 180.;
    const double angle_deg = angle_rad * PI_DEGS / M_PI;
    const double angle_exp = i <= PI_DEGS ? i : PI_DEGS - (i - PI_DEGS);
    constexpr double TOL = 10E-14;
    EXPECT_NEAR(angle_exp, angle_deg, TOL);
  }
}

TYPED_TEST(DCurveTests, SinglePointCurveConstruction) {
  const std::vector<TypeParam> points = DCurveCircle<TypeParam>::points(
      DCurveTests<TypeParam>::REF_FRAME,
      DCurveTests<TypeParam>::PNT_FRAME);
  const DCurve<TypeParam> curve_a({points[0]});

  EXPECT_EQ(curve_a.curve_length(), 0);
  EXPECT_THROW(curve_a.point_at(0), AssertException);
}

TYPED_TEST(DCurveTests, InvalidQueries) {
  constexpr double OVERFLOW = 0.1;
  DCurve<TypeParam> curve_a(DCurveCircle<TypeParam>::points(
      DCurveTests<TypeParam>::REF_FRAME,
      DCurveTests<TypeParam>::PNT_FRAME));
  EXPECT_THROW(
      {
        const auto invalid_point_a = curve_a.point_at(-OVERFLOW);
        (void)invalid_point_a;  // Avoid unused variable errors.
      },
      AssertException);
  EXPECT_THROW(
      {
        const auto invalid_point_b =
            curve_a.point_at(curve_a.curve_length() + OVERFLOW);
        (void)invalid_point_b;  // Avoid unused variable errors.
      },
      AssertException);
}

TEST(DCurveFSE3Tests, CheckReferenceFrame) {
  const DCurve<FSE3> curve_a(
      DCurveCircle<FSE3>::points(DCurveTests<FSE3>::REF_FRAME));
  EXPECT_EQ(curve_a.reference_frame(), DCurveTests<FSE3>::REF_FRAME);
}

}  // namespace resim::curves
