// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/header_from_ros2.hh"

#include <gtest/gtest.h>

#include <random>
#include <std_msgs/msg/header.hpp>

namespace resim::msg {

TEST(HeaderFromRos2Test, TestRoundTrip) {
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};
  constexpr std::size_t UB = 1000U;
  std::uniform_int_distribution<unsigned> dist{0U, UB};

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    std_msgs::msg::Header header;
    header.stamp.sec = static_cast<int>(dist(rng));
    header.stamp.nanosec = dist(rng);
    header.frame_id = "frame_" + std::to_string(dist(rng));

    const std_msgs::msg::Header round_trip_header =
        convert_to_ros2(convert_from_ros2(header));

    EXPECT_EQ(header.stamp.sec, round_trip_header.stamp.sec);
    EXPECT_EQ(header.stamp.nanosec, round_trip_header.stamp.nanosec);
    EXPECT_EQ(header.frame_id, round_trip_header.frame_id);
  }
}

}  // namespace resim::msg
