// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/ros2/odometry_from_ros2.hh"

#include <gtest/gtest.h>

#include "resim/msg/fuzz_helpers.hh"
#include "resim/msg/odometry.pb.h"
#include "resim/testing/fuzz_helpers.hh"
#include "resim/utils/inout.hh"

namespace resim::ros2 {

TEST(OdometryFromRos2Test, TestRoundTrip) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};
  const msg::Odometry test_odometry{random_element<msg::Odometry>(InOut{rng})};

  // ACTION
  const msg::Odometry round_tripped{
      convert_from_ros2(convert_to_ros2(test_odometry))};

  // VERIFICATION
  EXPECT_TRUE(verify_equality(test_odometry, round_tripped));
}

}  // namespace resim::ros2
