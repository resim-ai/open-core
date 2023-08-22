// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/transform_from_ros2.hh"

#include <gtest/gtest.h>

#include "resim/msg/fuzz_helpers.hh"
#include "resim/msg/transform.pb.h"
#include "resim/testing/fuzz_helpers.hh"
#include "resim/utils/inout.hh"

namespace resim::msg {

TEST(TransformFromRos2Test, TestRoundTrip) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};
  const TransformArray test_array{random_element<TransformArray>(InOut{rng})};
  // ACTION
  const TransformArray round_tripped{
      convert_from_ros2(convert_to_ros2(test_array))};

  // VERIFICATION
  EXPECT_TRUE(verify_equality(test_array, round_tripped));
}

}  // namespace resim::msg
