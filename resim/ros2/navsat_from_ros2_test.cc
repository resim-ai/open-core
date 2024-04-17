// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/ros2/navsat_from_ros2.hh"

#include <gtest/gtest.h>

#include "resim/converter/fuzz_helpers.hh"
#include "resim/msg/fuzz_helpers.hh"
#include "resim/msg/navsat.pb.h"
#include "resim/utils/inout.hh"

namespace resim::ros2 {

TEST(NavSatFixFromRos2Test, TestRoundTrip) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  // Do a lot of tests so we cover all switch branches.
  constexpr int NUM_TESTS = 500;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const msg::NavSatFix test_nav_sat_fix{
        converter::random_element<msg::NavSatFix>(InOut{rng})};

    // ACTION
    const msg::NavSatFix round_tripped{
        convert_from_ros2(convert_to_ros2(test_nav_sat_fix))};

    // VERIFICATION
    EXPECT_TRUE(converter::verify_equality(test_nav_sat_fix, round_tripped));
  }
}

TEST(NavSatFixFromRos2Test, TestBadStatuses) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  msg::NavSatFix nav_sat_fix{
      converter::random_element<msg::NavSatFix>(InOut{rng})};
  auto nav_sat_fix_ros2{convert_to_ros2(nav_sat_fix)};

  constexpr int BAD_VALUE = 100;
  nav_sat_fix.set_status(static_cast<msg::NavSatFix::Status>(BAD_VALUE));
  nav_sat_fix_ros2.status.status = BAD_VALUE;

  // ACTION / VERIFICATION
  EXPECT_THROW(convert_to_ros2(nav_sat_fix), AssertException);
  EXPECT_THROW(convert_from_ros2(nav_sat_fix_ros2), AssertException);
}

TEST(NavSatFixFromRos2Test, TestBadCovarianceTypes) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  msg::NavSatFix nav_sat_fix{
      converter::random_element<msg::NavSatFix>(InOut{rng})};
  auto nav_sat_fix_ros2{convert_to_ros2(nav_sat_fix)};

  constexpr int BAD_VALUE = 100;
  nav_sat_fix.set_position_covariance_type(
      static_cast<msg::NavSatFix::CovarianceType>(BAD_VALUE));
  nav_sat_fix_ros2.position_covariance_type = BAD_VALUE;

  // ACTION / VERIFICATION
  EXPECT_THROW(convert_to_ros2(nav_sat_fix), AssertException);
  EXPECT_THROW(convert_from_ros2(nav_sat_fix_ros2), AssertException);
}

}  // namespace resim::ros2
