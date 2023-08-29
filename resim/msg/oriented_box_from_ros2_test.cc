// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/oriented_box_from_ros2.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/geometry/proto/fuzz_helpers.hh"
#include "resim/geometry/proto/oriented_box.pb.h"
#include "resim/testing/fuzz_helpers.hh"
#include "resim/transforms/frame.hh"
#include "resim/utils/inout.hh"

namespace resim::msg {

TEST(OrientedBoxSE3FromRos2Test, TestRoundTrip) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};
  geometry::proto::OrientedBoxSE3 test_oriented_box{
      random_element<geometry::proto::OrientedBoxSE3>(InOut{rng})};
  // We don't use frame IDs when converting to/from ROS2.
  constexpr int DIMS = 3;
  test_oriented_box.mutable_reference_from_box()
      ->mutable_into()
      ->mutable_id()
      ->set_data(transforms::Frame<DIMS>::null_frame().id().to_string());
  test_oriented_box.mutable_reference_from_box()
      ->mutable_from()
      ->mutable_id()
      ->set_data(transforms::Frame<DIMS>::null_frame().id().to_string());

  // ACTION
  const geometry::proto::OrientedBoxSE3 round_tripped{
      convert_from_ros2(convert_to_ros2(test_oriented_box))};

  // VERIFICATION
  EXPECT_TRUE(verify_equality(test_oriented_box, round_tripped));
}

}  // namespace resim::msg
