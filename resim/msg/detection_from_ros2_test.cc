// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/detection_from_ros2.hh"

#include <gtest/gtest.h>

#include "resim/msg/detection.pb.h"
#include "resim/msg/fuzz_helpers.hh"
#include "resim/testing/fuzz_helpers.hh"
#include "resim/utils/inout.hh"

namespace resim::msg {

TEST(Detection3DFromRos2Test, TestRoundTrip) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};
  const Detection3D test_detection{random_element<Detection3D>(InOut{rng})};

  // ACTION
  const Detection3D round_tripped{
      convert_from_ros2(convert_to_ros2(test_detection))};

  // VERIFICATION
  EXPECT_TRUE(verify_equality(test_detection, round_tripped));
}

}  // namespace resim::msg
