
#include "resim/visualization/curve/line_primitive_from_t_curve.hh"

#include <foxglove/Color.pb.h>
#include <foxglove/LinePrimitive.pb.h>
#include <foxglove/Point3.pb.h>
#include <foxglove/Pose.pb.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cstdint>

#include "resim/assert/assert.hh"
#include "resim/curves/t_curve.hh"
#include "resim/curves/test_helpers.hh"
#include "resim/time/sample_interval.hh"
#include "resim/transforms/se3.hh"
#include "resim/visualization/color.hh"
#include "resim/visualization/curve/line_primitive_from_curve_test_helpers.hh"

namespace resim::visualization::curve {

using transforms::SE3;
using TCurve = curves::TCurve<SE3>;
using Vec3 = Eigen::Vector3d;

namespace {

// A simple helper to compare the points in the line primitive with what they
// are expected to be.
void expect_points_equal(
    const ::foxglove::LinePrimitive &line,
    const TCurve &curve,
    const double sample_period) {
  const int64_t expected_num_points =
      time::num_samples(curve.start_time(), curve.end_time(), sample_period);
  EXPECT_EQ(line.points().size(), expected_num_points);

  int jj = 0;
  time::sample_interval(
      curve.start_time(),
      curve.end_time(),
      sample_period,
      [&](const double time) {
        const ::foxglove::Point3 &point{line.points(jj)};
        const Vec3 expected_point{
            curve.point_at(time).frame_from_ref().inverse().translation()};
        EXPECT_DOUBLE_EQ(point.x(), expected_point.x());
        EXPECT_DOUBLE_EQ(point.y(), expected_point.y());
        EXPECT_DOUBLE_EQ(point.z(), expected_point.z());
        ++jj;
      });
}

}  // namespace

TEST(LinePrimitiveFromCurveTest, TestSampleCircle) {
  // SETUP
  const LinePrimitiveOptions options{
      .thickness = 0.01,
      .scale_invariant = false,
      .color = colors::SALMON,
  };
  const TCurve curve{curves::testing::make_circle_curve()};

  constexpr double SAMPLE_PERIOD = 1.5e-2;

  // ACTION
  ::foxglove::LinePrimitive line;
  line_primitive_from_t_curve(curve, &line, SAMPLE_PERIOD, options);

  // VERIFICATION
  EXPECT_EQ(line.type(), ::foxglove::LinePrimitive::LINE_STRIP);
  expect_pose_identity(line.pose());
  EXPECT_EQ(line.thickness(), options.thickness);
  EXPECT_EQ(line.scale_invariant(), options.scale_invariant);
  expect_points_equal(line, curve, SAMPLE_PERIOD);
  expect_colors_equal(line.color(), options.color);
  EXPECT_EQ(line.colors().size(), 0U);
  EXPECT_EQ(line.indices().size(), 0U);
}

TEST(LinePrimitiveFromCurveDeathTest, TestNullPtr) {
  const TCurve curve{curves::testing::make_circle_curve()};
  constexpr double SAMPLE_PERIOD = 1.5e-2;
  EXPECT_THROW(
      line_primitive_from_t_curve(curve, nullptr, SAMPLE_PERIOD),
      AssertException);
}

}  // namespace resim::visualization::curve
