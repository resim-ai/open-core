// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/fuzz_helpers.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/geometry/proto/fuzz_helpers.hh"
#include "resim/msg/header.pb.h"
#include "resim/msg/odometry.pb.h"
#include "resim/msg/pose.pb.h"
#include "resim/msg/transform.pb.h"

namespace resim::msg {

TEST(FuzzHelpersTest, TestHeaderEqual) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  const Header header{random_element<Header>(InOut{rng})};
  Header header_different_secs{header};
  header_different_secs.mutable_stamp()->set_seconds(
      header.stamp().seconds() + 1);
  Header header_different_nanos{header};
  header_different_nanos.mutable_stamp()->set_nanos(header.stamp().nanos() + 1);
  Header header_different_frame{header};
  header_different_frame.set_frame_id(header.frame_id() + "_different");

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(header, header));
  EXPECT_FALSE(verify_equality(header, header_different_secs));
  EXPECT_FALSE(verify_equality(header, header_different_nanos));
  EXPECT_FALSE(verify_equality(header, header_different_frame));
  EXPECT_FALSE(verify_equality(header_different_secs, header));
  EXPECT_FALSE(verify_equality(header_different_nanos, header));
  EXPECT_FALSE(verify_equality(header_different_frame, header));
}

TEST(FuzzHelpersTest, TestTransformStampedEqual) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  const TransformStamped transform{
      random_element<TransformStamped>(InOut{rng})};

  TransformStamped transform_different_header{transform};
  transform_different_header.mutable_header()->CopyFrom(
      random_element<Header>(InOut{rng}));

  TransformStamped transform_different_child{transform};
  transform_different_child.set_child_frame_id(
      transform.child_frame_id() + "_different");

  TransformStamped transform_different_transform{transform};
  transform_different_transform.mutable_transform()->CopyFrom(
      random_element<TransformStamped>(InOut{rng}).transform());

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(transform, transform));
  EXPECT_FALSE(verify_equality(transform, transform_different_header));
  EXPECT_FALSE(verify_equality(transform, transform_different_child));
  EXPECT_FALSE(verify_equality(transform, transform_different_transform));
  EXPECT_FALSE(verify_equality(transform_different_header, transform));
  EXPECT_FALSE(verify_equality(transform_different_child, transform));
  EXPECT_FALSE(verify_equality(transform_different_transform, transform));
}

TEST(FuzzHelpersTest, TestTransformArrayEqual) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  const TransformArray transform_array{
      random_element<TransformArray>(InOut{rng})};

  TransformArray transform_array_different_size{transform_array};
  ASSERT_GT(transform_array_different_size.transforms_size(), 0U);
  transform_array_different_size.mutable_transforms()->erase(
      transform_array_different_size.transforms().begin());

  TransformArray transform_array_different_element{transform_array};
  ASSERT_GT(transform_array_different_element.transforms_size(), 0U);
  transform_array_different_element.mutable_transforms(0)->CopyFrom(
      random_element<TransformStamped>(InOut{rng}));

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(transform_array, transform_array));
  EXPECT_FALSE(
      verify_equality(transform_array, transform_array_different_size));
  EXPECT_FALSE(
      verify_equality(transform_array, transform_array_different_element));
  EXPECT_FALSE(
      verify_equality(transform_array_different_size, transform_array));
  EXPECT_FALSE(
      verify_equality(transform_array_different_element, transform_array));
}

TEST(FuzzHelpersTest, TestPoseWithCovarianceEqual) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  const PoseWithCovariance pose_with_covariance{
      random_element<PoseWithCovariance>(InOut{rng})};

  PoseWithCovariance pose_with_covariance_different_pose{pose_with_covariance};
  pose_with_covariance_different_pose.mutable_pose()->CopyFrom(
      random_element<PoseWithCovariance>(InOut{rng}).pose());

  PoseWithCovariance pose_with_covariance_different_covariance{
      pose_with_covariance};
  pose_with_covariance_different_covariance.mutable_covariance()->Set(
      0,
      -pose_with_covariance.covariance(0));

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(pose_with_covariance, pose_with_covariance));
  EXPECT_FALSE(verify_equality(
      pose_with_covariance,
      pose_with_covariance_different_pose));
  EXPECT_FALSE(verify_equality(
      pose_with_covariance,
      pose_with_covariance_different_covariance));
  EXPECT_FALSE(verify_equality(
      pose_with_covariance_different_pose,
      pose_with_covariance));
  EXPECT_FALSE(verify_equality(
      pose_with_covariance_different_covariance,
      pose_with_covariance));
}

TEST(FuzzHelpersTest, TestTwistEqual) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  const Twist twist{random_element<Twist>(InOut{rng})};

  Twist twist_different_algebra{twist};
  twist_different_algebra.mutable_algebra()->Set(0, -twist.algebra(0));

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(twist, twist));
  EXPECT_FALSE(verify_equality(twist, twist_different_algebra));
  EXPECT_FALSE(verify_equality(twist_different_algebra, twist));
}

TEST(FuzzHelpersTest, TestTwistWithCovarianceEqual) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  const TwistWithCovariance twist_with_covariance{
      random_element<TwistWithCovariance>(InOut{rng})};

  TwistWithCovariance twist_with_covariance_different_twist{
      twist_with_covariance};
  twist_with_covariance_different_twist.mutable_twist()->CopyFrom(
      random_element<Twist>(InOut{rng}));

  TwistWithCovariance twist_with_covariance_different_covariance{
      twist_with_covariance};
  twist_with_covariance_different_covariance.mutable_covariance()->Set(
      0,
      -twist_with_covariance.covariance(0));

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(twist_with_covariance, twist_with_covariance));
  EXPECT_FALSE(verify_equality(
      twist_with_covariance,
      twist_with_covariance_different_twist));
  EXPECT_FALSE(verify_equality(
      twist_with_covariance,
      twist_with_covariance_different_covariance));
  EXPECT_FALSE(verify_equality(
      twist_with_covariance_different_twist,
      twist_with_covariance));
  EXPECT_FALSE(verify_equality(
      twist_with_covariance_different_covariance,
      twist_with_covariance));
}

TEST(FuzzHelpersTest, TestOdometryEqual) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  const Odometry odometry{random_element<Odometry>(InOut{rng})};

  Odometry odometry_different_header{odometry};
  odometry_different_header.mutable_header()->CopyFrom(
      random_element<Header>(InOut{rng}));

  Odometry odometry_different_child_frame_id{odometry};
  odometry_different_child_frame_id.set_child_frame_id(
      odometry.child_frame_id() + "_different");

  Odometry odometry_different_pose{odometry};
  odometry_different_pose.mutable_pose()->CopyFrom(
      random_element<PoseWithCovariance>(InOut{rng}));

  Odometry odometry_different_twist{odometry};
  odometry_different_twist.mutable_twist()->CopyFrom(
      random_element<TwistWithCovariance>(InOut{rng}));

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(odometry, odometry));
  EXPECT_FALSE(verify_equality(odometry, odometry_different_header));
  EXPECT_FALSE(verify_equality(odometry, odometry_different_child_frame_id));
  EXPECT_FALSE(verify_equality(odometry, odometry_different_pose));
  EXPECT_FALSE(verify_equality(odometry_different_twist, odometry));
  EXPECT_FALSE(verify_equality(odometry_different_child_frame_id, odometry));
  EXPECT_FALSE(verify_equality(odometry_different_pose, odometry));
  EXPECT_FALSE(verify_equality(odometry_different_twist, odometry));
}

TEST(FuzzHelpersTest, TestDetectionEqual) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  const Detection3D detection{random_element<Detection3D>(InOut{rng})};

  Detection3D detection_different_header{detection};
  detection_different_header.mutable_header()->CopyFrom(
      random_element<Header>(InOut{rng}));

  Detection3D detection_different_bbox{detection};
  detection_different_bbox.mutable_bbox()->CopyFrom(
      random_element<geometry::proto::OrientedBoxSE3>(InOut{rng}));

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(detection, detection));
  EXPECT_FALSE(verify_equality(detection, detection_different_header));
  EXPECT_FALSE(verify_equality(detection, detection_different_bbox));
  EXPECT_FALSE(verify_equality(detection_different_bbox, detection));
}

TEST(FuzzHelpersTest, TestNavSatFixEqual) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  const NavSatFix nav_sat_fix{random_element<NavSatFix>(InOut{rng})};

  NavSatFix nav_sat_fix_different_header{nav_sat_fix};
  nav_sat_fix_different_header.mutable_header()->CopyFrom(
      random_element<Header>(InOut{rng}));

  constexpr int NUM_STATUSES = 4;
  NavSatFix nav_sat_fix_different_status{nav_sat_fix};
  nav_sat_fix_different_status.set_status(static_cast<NavSatFix::Status>(
      (static_cast<int>(nav_sat_fix.status() + 1) % NUM_STATUSES)));

  NavSatFix nav_sat_fix_different_latitude{nav_sat_fix};
  nav_sat_fix_different_latitude.set_latitude_deg(-nav_sat_fix.latitude_deg());

  NavSatFix nav_sat_fix_different_longitude{nav_sat_fix};
  nav_sat_fix_different_longitude.set_longitude_deg(
      -nav_sat_fix.longitude_deg());

  NavSatFix nav_sat_fix_different_altitude_m{nav_sat_fix};
  nav_sat_fix_different_altitude_m.set_altitude_m(-nav_sat_fix.altitude_m());

  NavSatFix nav_sat_fix_different_covariance{nav_sat_fix};
  nav_sat_fix_different_covariance.mutable_position_covariance_m2()->Set(
      0,
      -nav_sat_fix.position_covariance_m2(0));

  constexpr int NUM_COV_TYPES = 4;
  NavSatFix nav_sat_fix_different_covariance_type{nav_sat_fix};
  nav_sat_fix_different_covariance_type.set_position_covariance_type(
      static_cast<NavSatFix::CovarianceType>(
          (static_cast<int>(nav_sat_fix.position_covariance_type() + 1) %
           NUM_COV_TYPES)));

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(nav_sat_fix, nav_sat_fix));
  EXPECT_FALSE(verify_equality(nav_sat_fix, nav_sat_fix_different_header));
  EXPECT_FALSE(verify_equality(nav_sat_fix, nav_sat_fix_different_status));
  EXPECT_FALSE(verify_equality(nav_sat_fix, nav_sat_fix_different_latitude));
  EXPECT_FALSE(verify_equality(nav_sat_fix, nav_sat_fix_different_longitude));
  EXPECT_FALSE(verify_equality(nav_sat_fix, nav_sat_fix_different_altitude_m));
  EXPECT_FALSE(verify_equality(nav_sat_fix, nav_sat_fix_different_covariance));
  EXPECT_FALSE(
      verify_equality(nav_sat_fix, nav_sat_fix_different_covariance_type));
  EXPECT_FALSE(verify_equality(nav_sat_fix_different_header, nav_sat_fix));
  EXPECT_FALSE(verify_equality(nav_sat_fix_different_status, nav_sat_fix));
  EXPECT_FALSE(verify_equality(nav_sat_fix_different_latitude, nav_sat_fix));
  EXPECT_FALSE(verify_equality(nav_sat_fix_different_longitude, nav_sat_fix));
  EXPECT_FALSE(verify_equality(nav_sat_fix_different_altitude_m, nav_sat_fix));
  EXPECT_FALSE(verify_equality(nav_sat_fix_different_covariance, nav_sat_fix));
  EXPECT_FALSE(
      verify_equality(nav_sat_fix_different_covariance_type, nav_sat_fix));
}

}  // namespace resim::msg
