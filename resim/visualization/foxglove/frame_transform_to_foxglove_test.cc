
#include "resim/visualization/foxglove/frame_transform_to_foxglove.hh"

#include <foxglove/FrameTransform.pb.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <ratio>

#include "resim/time/proto/time_to_proto.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/liegroup_test_helpers.hh"
#include "resim/transforms/se3.hh"
#include "resim/visualization/foxglove/orientation_to_foxglove.hh"
#include "resim/visualization/foxglove/vector_to_foxglove.hh"

namespace resim::visualization::foxglove {

using Frame = transforms::Frame<transforms::SE3::DIMS>;

// Helper function to compare the packed msg to the original pose.
// NOLINTBEGIN(readability-function-cognitive-complexity)
void expect_poses_match(
    const transforms::SE3 &pose,
    const ::foxglove::FrameTransform &msg) {
  const Eigen::Quaterniond quat{pose.rotation().quaternion()};
  EXPECT_EQ(quat.w(), msg.rotation().w());
  EXPECT_EQ(quat.x(), msg.rotation().x());
  EXPECT_EQ(quat.y(), msg.rotation().y());
  EXPECT_EQ(quat.z(), msg.rotation().z());
  EXPECT_EQ(pose.translation().x(), msg.translation().x());
  EXPECT_EQ(pose.translation().y(), msg.translation().y());
  EXPECT_EQ(pose.translation().z(), msg.translation().z());
}
// NOLINTEND(readability-function-cognitive-complexity)

TEST(FrameTransformToFoxgloveTest, TestPackIntoFoxglove) {
  // SETUP
  std::vector<transforms::SE3> test_elements{
      transforms::make_test_group_elements<transforms::SE3>()};

  for (transforms::SE3 &element : test_elements) {
    ::foxglove::FrameTransform frame_transform;
    constexpr auto PARENT_NAME = "parent";
    constexpr auto CHILD_NAME = "child";
    constexpr time::Timestamp TIME{std::chrono::seconds(1)};

    element.set_frames(Frame::new_frame(), Frame::new_frame());

    // ACTION
    pack_into_foxglove(
        element,
        TIME,
        &frame_transform,
        PARENT_NAME,
        CHILD_NAME);

    // VERIFICATION
    const auto &time_msg = frame_transform.timestamp();
    EXPECT_EQ(
        time_msg.seconds() * std::nano::den + time_msg.nanos(),
        TIME.time_since_epoch().count());

    EXPECT_EQ(frame_transform.parent_frame_id(), PARENT_NAME);
    EXPECT_EQ(frame_transform.child_frame_id(), CHILD_NAME);

    expect_poses_match(element, frame_transform);
  }
}

TEST(FrameTransformToFoxgloveTest, TestPackIntoFoxgloveNoFrameIds) {
  // SETUP
  std::vector<transforms::SE3> test_elements{
      transforms::make_test_group_elements<transforms::SE3>()};

  for (transforms::SE3 &element : test_elements) {
    ::foxglove::FrameTransform frame_transform;
    constexpr time::Timestamp TIME{std::chrono::seconds(1)};
    element.set_frames(Frame::new_frame(), Frame::new_frame());

    // ACTION
    pack_into_foxglove(element, TIME, &frame_transform);

    // VERIFICATION
    const auto &time_msg = frame_transform.timestamp();
    EXPECT_EQ(
        time_msg.seconds() * std::nano::den + time_msg.nanos(),
        TIME.time_since_epoch().count());

    EXPECT_EQ(
        frame_transform.parent_frame_id(),
        element.into().id().to_string());
    EXPECT_EQ(
        frame_transform.child_frame_id(),
        element.from().id().to_string());

    expect_poses_match(element, frame_transform);
  }
}

TEST(FrameTransformToFoxgloveTest, TestUnframedThrows) {}

}  // namespace resim::visualization::foxglove
