
#include "resim/visualization/foxglove/orientation_to_foxglove.hh"

#include <foxglove/Quaternion.pb.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "resim/testing/random_matrix.hh"
#include "resim/transforms/so3.hh"

namespace resim::visualization::foxglove {

TEST(OrientationToFoxgloveTest, TestPackIntoFoxglove) {
  // SETUP
  constexpr unsigned SEED = 84U;
  std::mt19937 rng{SEED};

  constexpr int NUM_TESTS = 1000;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Eigen::Quaterniond quat{testing::random_quaternion(rng)};
    const transforms::SO3 random_orientation{quat};

    // ACTION
    ::foxglove::Quaternion quat_msg;
    pack_into_foxglove(random_orientation, &quat_msg);

    // VERIFICATION
    // This may differ from quat above by a minus sign since q and -q represent
    // the same orientation.
    const Eigen::Quaterniond expected_quat{random_orientation.quaternion()};
    EXPECT_EQ(expected_quat.w(), quat_msg.w());
    EXPECT_EQ(expected_quat.x(), quat_msg.x());
    EXPECT_EQ(expected_quat.y(), quat_msg.y());
    EXPECT_EQ(expected_quat.z(), quat_msg.z());
  }
}

}  // namespace resim::visualization::foxglove
