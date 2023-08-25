// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/pose_from_ros2.hh"

#include <gtest/gtest.h>

#include "resim/msg/fuzz_helpers.hh"
#include "resim/msg/pose.pb.h"
#include "resim/testing/fuzz_helpers.hh"
#include "resim/utils/inout.hh"

namespace resim::msg {

using Types = ::testing::Types<PoseWithCovariance, Twist, TwistWithCovariance>;

template <typename T>
struct PoseFromRos2Test : public ::testing::Test {};

TYPED_TEST_SUITE(PoseFromRos2Test, Types);

TYPED_TEST(PoseFromRos2Test, TestRoundTrip) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};
  const TypeParam test_element{random_element<TypeParam>(InOut{rng})};

  // ACTION
  const TypeParam round_tripped{
      convert_from_ros2(convert_to_ros2(test_element))};

  // VERIFICATION
  EXPECT_TRUE(verify_equality(test_element, round_tripped));
}

}  // namespace resim::msg