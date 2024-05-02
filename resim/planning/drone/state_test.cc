// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/planning/drone/state.hh"

#include <gtest/gtest.h>

#include "resim/math/is_approx.hh"
#include "resim/testing/random_matrix.hh"

namespace resim::planning::drone {

TEST(StateTest, TestAddAndSubtract) {
  // SETUP
  constexpr size_t SEED = 93U;
  std::mt19937 rng{SEED};
  constexpr int NUM_TESTS = 100;
  const State state;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const State::Vec delta{testing::random_vector<State::Vec>(rng)};

    // ACTION
    const State sum{state + delta};

    // VERIFICATION
    EXPECT_FALSE(
        sum.scene_from_body_rotation.is_approx(state.scene_from_body_rotation));
    EXPECT_FALSE(math::is_approx(sum.position, state.position));
    EXPECT_FALSE(math::is_approx(sum.angular_velocity, state.angular_velocity));
    EXPECT_FALSE(math::is_approx(sum.velocity, state.velocity));

    // ACTION
    const State::Vec diff{sum - state};

    // VERIFICATION
    EXPECT_TRUE(math::is_approx(diff, delta));
  }
}

}  // namespace resim::planning::drone
