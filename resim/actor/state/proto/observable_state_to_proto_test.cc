#include "resim/actor/state/proto/observable_state_to_proto.hh"

#include <gtest/gtest.h>

#include <algorithm>
#include <random>

#include "resim/actor/actor_id.hh"
#include "resim/actor/state/observable_state.hh"
#include "resim/actor/state/proto/observable_state.pb.h"
#include "resim/actor/state/rigid_body_state.hh"
#include "resim/assert/assert.hh"
#include "resim/curves/two_jet.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"

namespace resim::actor::state {
namespace {

// Make a single random test observable state.
template <typename RNG>
ObservableState make_test_state(RNG &&rng) {
  std::uniform_real_distribution<double> dist{-1., 1.};
  return ObservableState{
      .id = ActorId::new_uuid(),
      .is_spawned = dist(rng) > 0.,
      .time_of_validity = time::Timestamp{time::as_duration(dist(rng))},
      .state =
          RigidBodyState<transforms::SE3>{
              curves::TwoJetR<transforms::SE3>{
                  transforms::SE3::exp(
                      testing::random_vector<transforms::SE3::TangentVector>(
                          rng,
                          dist)),
                  testing::random_vector<transforms::SE3::TangentVector>(
                      rng,
                      dist),
                  testing::random_vector<transforms::SE3::TangentVector>(
                      rng,
                      dist)},
          },
  };
}

// Make a a vector of test observable state.
template <typename RNG>
std::vector<ObservableState> make_test_states(RNG &&rng) {
  std::vector<ObservableState> result;
  constexpr int NUM_STATES = 10;
  result.reserve(NUM_STATES);
  for (int ii = 0; ii < NUM_STATES; ++ii) {
    result.emplace_back(make_test_state(rng));
  }
  return result;
}

// Compare two observable states for equality.
bool states_equal(const ObservableState &a, const ObservableState &b) {
  return a.id == b.id and a.is_spawned == b.is_spawned &&
         a.time_of_validity == b.time_of_validity &&
         a.state.ref_from_body_two_jet().is_approx(
             b.state.ref_from_body_two_jet());
}

// Compare two vectors of observable states for equality.
bool states_equal(
    const std::vector<ObservableState> &a,
    const std::vector<ObservableState> &b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (int ii = 0; ii < a.size(); ++ii) {
    if (not states_equal(a.at(ii), b.at(ii))) {
      return false;
    }
  }
  return true;
}

}  // namespace

TEST(ObservableStateToProtoTest, TestRoundTrip) {
  // SETUP
  constexpr std::size_t SEED = 9143U;
  std::mt19937 rng{SEED};

  constexpr int NUM_TESTS = 10;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const ObservableState state{make_test_state(rng)};

    // ACTION
    proto::ObservableState state_msg;
    pack(state, &state_msg);
    const ObservableState unpacked{unpack(state_msg)};

    // VERIFICATION
    EXPECT_TRUE(states_equal(state, unpacked));
  }
}

TEST(ObservableStateToProtoTest, ThrowOnNullptr) {
  proto::ObservableState *const bad_msg_ptr{nullptr};
  EXPECT_THROW(pack(ObservableState{}, bad_msg_ptr), AssertException);
}

TEST(ObservableStatesToProtoTest, TestRoundTrip) {
  // SETUP
  constexpr std::size_t SEED = 9143U;
  std::mt19937 rng{SEED};

  constexpr int NUM_TESTS = 10;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const std::vector<ObservableState> states{make_test_states(rng)};

    // ACTION
    proto::ObservableStates states_msg;
    pack(states, &states_msg);
    const std::vector<ObservableState> unpacked{unpack(states_msg)};

    // VERIFICATION
    EXPECT_TRUE(states_equal(states, unpacked));
  }
}

TEST(ObservableStatesToProtoTest, ThrowOnNullptr) {
  proto::ObservableStates *const bad_msg_ptr{nullptr};
  EXPECT_THROW(
      pack(std::vector<ObservableState>(), bad_msg_ptr),
      AssertException);
}
}  // namespace resim::actor::state
