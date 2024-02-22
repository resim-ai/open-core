// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/foxglove/wireframe_to_foxglove.hh"

#include <foxglove/LinePrimitive.pb.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "resim/geometry/drone_wireframe.hh"
#include "resim/geometry/wireframe.hh"
#include "resim/visualization/color.hh"

namespace resim::visualization::foxglove {

namespace {

constexpr double CHASSIS_RADIUS_M = 1.;
constexpr double ROTOR_LATERAL_OFFSET_M = 0.3;
constexpr double ROTOR_VERTICAL_OFFSET_M = 0.3;
constexpr double ROTOR_RADIUS_M = 0.5;
constexpr std::size_t SAMPLES_PER_ROTOR = 2;

const geometry::DroneExtents test_extents{
    .chassis_radius_m = CHASSIS_RADIUS_M,
    .rotor_lateral_offset_m = ROTOR_LATERAL_OFFSET_M,
    .rotor_vertical_offset_m = ROTOR_VERTICAL_OFFSET_M,
    .rotor_radius_m = ROTOR_RADIUS_M,
    .samples_per_rotor = SAMPLES_PER_ROTOR,
};

void expect_points_equal(
    const Eigen::Vector3d &point,
    const ::foxglove::Point3 &point_msg) {
  EXPECT_DOUBLE_EQ(point.x(), point_msg.x());
  EXPECT_DOUBLE_EQ(point.y(), point_msg.y());
  EXPECT_DOUBLE_EQ(point.z(), point_msg.z());
}

void expect_colors_equal(
    const Color &color,
    const ::foxglove::Color &color_msg) {
  EXPECT_EQ(color.r, color_msg.r());
  EXPECT_EQ(color.g, color_msg.g());
  EXPECT_EQ(color.b, color_msg.b());
  EXPECT_EQ(color.a, color_msg.a());
}

void expect_pose_identity(const ::foxglove::Pose &pose) {
  EXPECT_EQ(pose.position().x(), 0.0);
  EXPECT_EQ(pose.position().y(), 0.0);
  EXPECT_EQ(pose.position().z(), 0.0);

  EXPECT_EQ(pose.orientation().w(), 1.0);
  EXPECT_EQ(pose.orientation().x(), 0.0);
  EXPECT_EQ(pose.orientation().y(), 0.0);
  EXPECT_EQ(pose.orientation().z(), 0.0);
}

}  // namespace

TEST(WireframeToFoxgloveTest, TestWireframeToFoxglove) {
  // SETUP
  const geometry::Wireframe wireframe{geometry::drone_wireframe(test_extents)};

  // ACTION
  ::foxglove::LinePrimitive out;
  pack_into_foxglove(wireframe, &out);

  // VERIFICATION
  EXPECT_EQ(out.type(), ::foxglove::LinePrimitive::LINE_LIST);

  // TODO(michael) Put this in a more common library
  expect_pose_identity(out.pose());
  constexpr double DEFAULT_THICKNESS = 2.;
  constexpr Color DEFAULT_COLOR = colors::CHARTREUSE;
  EXPECT_EQ(out.thickness(), DEFAULT_THICKNESS);
  EXPECT_TRUE(out.scale_invariant());
  expect_colors_equal(DEFAULT_COLOR, out.color());

  ASSERT_EQ(out.points().size(), 2 * wireframe.edges().size());

  for (int ii = 0U; ii < wireframe.edges().size(); ++ii) {
    const Eigen::Vector3d &p1{
        wireframe.points().at(wireframe.edges().at(ii)[0])};
    const Eigen::Vector3d &p2{
        wireframe.points().at(wireframe.edges().at(ii)[1])};
    expect_points_equal(p1, out.points().at(2 * ii));
    expect_points_equal(p2, out.points().at(2 * ii + 1));
  }

  EXPECT_EQ(out.colors().size(), 0U);
  EXPECT_EQ(out.indices().size(), 0U);
}

}  // namespace resim::visualization::foxglove
