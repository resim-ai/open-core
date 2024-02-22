
#include "resim/visualization/foxglove/pose_to_foxglove.hh"

#include <foxglove/Pose.pb.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <vector>

#include "resim/transforms/liegroup_test_helpers.hh"
#include "resim/transforms/se3.hh"

namespace resim::visualization::foxglove {
namespace {
using transforms::SE3;

// Helper function to compare the packed msg to the original pose.
// NOLINTBEGIN(readability-function-cognitive-complexity)
void expect_poses_match(const SE3 &pose, const ::foxglove::Pose &msg) {
  const Eigen::Quaterniond quat{pose.rotation().quaternion()};
  EXPECT_EQ(quat.w(), msg.orientation().w());
  EXPECT_EQ(quat.x(), msg.orientation().x());
  EXPECT_EQ(quat.y(), msg.orientation().y());
  EXPECT_EQ(quat.z(), msg.orientation().z());
  EXPECT_EQ(pose.translation().x(), msg.position().x());
  EXPECT_EQ(pose.translation().y(), msg.position().y());
  EXPECT_EQ(pose.translation().z(), msg.position().z());
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace

TEST(PoseToFoxgloveTest, TestPackIntoFoxglove) {
  // SETUP
  const std::vector<SE3> test_group_elements{
      transforms::make_test_group_elements<SE3>()};

  for (const SE3 &element : test_group_elements) {
    // ACTION
    ::foxglove::Pose msg;
    pack_into_foxglove(element, &msg);

    // VERIFICATION
    expect_poses_match(element, msg);
  }
}

}  // namespace resim::visualization::foxglove
