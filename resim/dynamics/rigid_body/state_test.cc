// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/dynamics/rigid_body/state.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/math/is_approx.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/frame.hh"

namespace resim::dynamics::rigid_body {

using transforms::SE3;
using Frame = transforms::Frame<transforms::SE3::DIMS>;
using TangentVector = SE3::TangentVector;

// Helper to get a randomly generated State.
template <typename RNG>
State random_state(RNG &&rng) {
  return State{
      .reference_from_body = SE3::exp(
          testing::random_vector<TangentVector>(rng),
          Frame::new_frame(),
          Frame::new_frame()),
      .d_reference_from_body = testing::random_vector<TangentVector>(rng),
  };
}

// Helper to get a randomly generated State delta.
template <typename RNG>
State::Delta random_delta(RNG &&rng) {
  return testing::random_vector<State::Delta>(rng);
}

// Test that adding gives the expected result.
TEST(StateTest, TestAdd) {
  constexpr unsigned SEED = 382U;
  std::mt19937 rng{SEED};

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    // SETUP
    const State state{random_state(rng)};
    const State::Delta delta{random_delta(rng)};

    // ACTION
    const State sum{state + delta};

    // VERIFICATION
    const SE3 prev_from_next{
        state.reference_from_body.inverse() * sum.reference_from_body};

    EXPECT_TRUE(math::is_approx(
        prev_from_next.log(),
        State::delta_vector_pose_part(delta)));
    EXPECT_TRUE(math::is_approx(
        sum.d_reference_from_body - state.d_reference_from_body,
        State::delta_vector_velocity_part(delta)));

    EXPECT_EQ(state.reference_from_body.into(), sum.reference_from_body.into());
    EXPECT_EQ(state.reference_from_body.from(), sum.reference_from_body.from());
  }
}

// Test that commuted addition works too
TEST(StateTest, TestAddCommuted) {
  constexpr unsigned SEED = 382U;
  std::mt19937 rng{SEED};

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    // SETUP
    const State state{random_state(rng)};
    const State::Delta delta{random_delta(rng)};

    // ACTION
    const State sum{state + delta};
    const State other_sum{delta + state};

    // VERIFICATION
    EXPECT_TRUE(
        sum.reference_from_body.is_approx(other_sum.reference_from_body));
    EXPECT_TRUE(math::is_approx(
        sum.d_reference_from_body,
        other_sum.d_reference_from_body));
  }
}

// Test that subtraction matches with our addition implementation
TEST(StateTest, TestSubtract) {
  constexpr unsigned SEED = 382U;
  std::mt19937 rng{SEED};

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    // SETUP
    const State state_a{random_state(rng)};
    State state_b{random_state(rng)};

    // These have to match
    state_b.reference_from_body.set_frames(
        state_a.reference_from_body.into(),
        state_a.reference_from_body.from());

    // ACTION
    const State should_match_b{(state_b - state_a) + state_a};

    // VERIFICATION
    EXPECT_TRUE(state_b.reference_from_body.is_approx(
        should_match_b.reference_from_body));
    EXPECT_TRUE(state_b.d_reference_from_body.isApprox(
        should_match_b.d_reference_from_body));
  }
}

}  // namespace resim::dynamics::rigid_body
