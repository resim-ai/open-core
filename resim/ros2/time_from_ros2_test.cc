// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/ros2/time_from_ros2.hh"

#include <gtest/gtest.h>

#include <builtin_interfaces/msg/time.hpp>
#include <random>

namespace resim::ros2 {

using Ros2Time = builtin_interfaces::msg::Time;

TEST(TimeFromRos2Test, TestRoundTrip) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};
  constexpr std::size_t UB = 1000U;
  std::uniform_int_distribution<unsigned> dist{0U, UB};

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    Ros2Time time;
    time.sec = static_cast<int>(dist(rng));
    time.nanosec = dist(rng);

    // ACTION
    const Ros2Time round_trip_time = convert_to_ros2(convert_from_ros2(time));

    // VERIFICATION
    EXPECT_EQ(time.sec, round_trip_time.sec);
    EXPECT_EQ(time.nanosec, round_trip_time.nanosec);
  }
}

}  // namespace resim::ros2
