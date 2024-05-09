// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/planning/drone/control.hh"

#include <gtest/gtest.h>

#include "resim/math/is_approx.hh"
#include "resim/testing/random_matrix.hh"

namespace resim::planning::drone {

TEST(ControlTest, TestAddAndSubtract) {
  // SETUP
  constexpr size_t SEED = 93U;
  std::mt19937 rng{SEED};
  constexpr int NUM_TESTS = 100;
  const Control control;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Control::Vec delta{testing::random_vector<Control::Vec>(rng)};

    // ACTION
    const Control sum{control + delta};

    // VERIFICATION
    EXPECT_FALSE(math::is_approx(
        sum.angular_acceleration,
        control.angular_acceleration));
    EXPECT_FALSE(math::is_approx(sum.thrust, control.thrust));

    // ACTION
    const Control::Vec diff{sum - control};

    // VERIFICATION
    EXPECT_TRUE(math::is_approx(diff, delta));
  }
}

}  // namespace resim::planning::drone
