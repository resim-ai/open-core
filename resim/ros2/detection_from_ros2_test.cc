// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/ros2/detection_from_ros2.hh"

#include <gtest/gtest.h>

#include "resim/converter/fuzz_helpers.hh"
#include "resim/msg/detection.pb.h"
#include "resim/msg/fuzz_helpers.hh"
#include "resim/utils/inout.hh"

namespace resim::ros2 {

using Types = ::testing::Types<
    msg::Detection3D,
    msg::Detection3DArray,
    msg::BoundingBox2D,
    msg::Detection2D,
    msg::Detection2DArray>;

template <typename T>
class Detection3DFromRos2Test : public ::testing::Test {};

TYPED_TEST_SUITE(Detection3DFromRos2Test, Types);

TYPED_TEST(Detection3DFromRos2Test, TestRoundTrip) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};
  const TypeParam test_element{
      converter::random_element<TypeParam>(InOut{rng})};

  // ACTION
  const TypeParam round_tripped{
      convert_from_ros2(convert_to_ros2(test_element))};

  // VERIFICATION
  EXPECT_TRUE(converter::verify_equality(test_element, round_tripped));
}

}  // namespace resim::ros2
