#include "min_distance.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <random>
#include <utility>

#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/actor/state/rigid_body_state.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/testing/random_matrix.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::metrics {

namespace {
using time::Timestamp;
using transforms::FSE3;
using transforms::SE3;
using Frame = transforms::Frame<FSE3::DIMS>;
using RigidBodyState = actor::state::RigidBodyState<FSE3>;
using actor::ActorId;
using actor::state::ObservableState;

constexpr double THRESHOLD = 1e-7;
bool is_near(double a, double b) { return abs(a - b) < THRESHOLD; }

const Frame TARGET_FRAME = Frame::new_frame();
const Frame REF_FRAME = Frame::new_frame();
const Frame TEST_FRAME_1 = Frame::new_frame();
const Frame TEST_FRAME_2 = Frame::new_frame();

constexpr Timestamp TIME{std::chrono::seconds(982)};
template <typename Rng>
FSE3 random_fse3(Rng&& rng) {
  return FSE3(
      SE3::exp(testing::random_vector<typename SE3::TangentVector>(
          std::forward<Rng>(rng))),
      REF_FRAME,
      Frame::new_frame());
}

ObservableState make_observable_state(
    const time::Timestamp& time,
    const FSE3& ref_from_body,
    const bool is_spawned = true) {
  ObservableState state{};
  state.id = ActorId::new_uuid();
  state.time_of_validity = time;
  state.state = RigidBodyState(ref_from_body);
  state.is_spawned = is_spawned;

  return state;
};

template <typename Rng>
ObservableState random_observable_state(Rng&& rng) {
  return make_observable_state(TIME, random_fse3(rng));
}

const ObservableState TARGET_STATE =
    make_observable_state(TIME, FSE3::identity(REF_FRAME, TARGET_FRAME));

const ObservableState TEST_STATE_1 = make_observable_state(
    TIME,
    FSE3(SE3(Eigen::Vector3d::UnitX()), REF_FRAME, TEST_FRAME_1));

const ObservableState TEST_STATE_2 = make_observable_state(
    TIME,
    FSE3(SE3(Eigen::Vector3d{0.0, 2.0, 0.0}), REF_FRAME, TEST_FRAME_2));

}  // namespace

TEST(MinDistanceTests, EmptyListWithState) {
  // SETUP
  const std::vector<ObservableState> empty_states;

  // TEST
  const std::optional<double> d = min_distance(TARGET_STATE, empty_states);

  // VERIFY
  EXPECT_EQ(d, std::nullopt);
};

TEST(MinDistanceTests, EmptyListWithID) {
  // SETUP
  const std::vector<ObservableState> empty_states;

  // TEST + VERIFY
  EXPECT_THROW(min_distance(TARGET_STATE.id, empty_states), AssertException);
}

TEST(MinDistanceTests, IDNotInList) {
  // SETUP
  constexpr std::size_t SEED = 694U;
  std::mt19937 rng{SEED};
  const std::vector<ObservableState> states{random_observable_state(rng)};

  // TEST + VERIFY
  EXPECT_THROW(min_distance(TARGET_STATE.id, states), AssertException);
}

TEST(MinDistanceTests, ShouldSkipSelf) {
  // SETUP
  constexpr std::size_t SEED = 950302U;
  std::mt19937 rng{SEED};
  const std::vector<ObservableState> states_without_self{
      random_observable_state(rng),
      random_observable_state(rng),
      random_observable_state(rng)};
  std::vector<ObservableState> states_with_self(states_without_self);
  states_with_self.push_back(TARGET_STATE);

  // TEST
  const std::optional<double> distance_without_self =
      min_distance(TARGET_STATE, states_without_self);
  const std::optional<double> distance_with_self =
      min_distance(TARGET_STATE, states_with_self);

  // VERIFY
  EXPECT_TRUE(distance_with_self.has_value());
  EXPECT_TRUE(distance_without_self.has_value());
  EXPECT_EQ(distance_without_self, distance_with_self);
}

TEST(MinDistanceTests, RandomStates) {
  // SETUP
  constexpr std::size_t SEED = 482984U;
  std::mt19937 rng{SEED};
  const std::vector<ObservableState> states{
      random_observable_state(rng),
      random_observable_state(rng),
      random_observable_state(rng),
      TARGET_STATE};

  // TEST
  const std::optional<double> min_distance_with_state =
      min_distance(TARGET_STATE, states);
  const std::optional<double> min_distance_without_state =
      min_distance(TARGET_STATE.id, states);

  // VERIFY
  EXPECT_TRUE(min_distance_with_state.has_value());
  EXPECT_TRUE(min_distance_without_state.has_value());

  // Optional values checked above
  // NOLINTBEGIN(bugprone-unchecked-optional-access)
  EXPECT_EQ(min_distance_with_state.value(), min_distance_without_state);

  size_t num_near = 0;
  for (const ObservableState& state : states) {
    if (state.id == TARGET_STATE.id) {
      continue;
    }
    const double state_distance = transforms::fse3_inverse_distance(
        state.state.ref_from_body(),
        TARGET_STATE.state.ref_from_body());
    // We expect all distances to be greater than or equal to the
    // min distance
    EXPECT_GE(state_distance, min_distance_with_state.value());

    // Count values "equal" to the min distance
    num_near += static_cast<size_t>(
        is_near(state_distance, min_distance_with_state.value()));
  }
  // NOLINTEND(bugprone-unchecked-optional-access)

  // We expect exactly one value to be "equal to the min distance"
  EXPECT_EQ(num_near, 1U);
}

TEST(MinDistanceTests, FixedStates) {
  // SETUP
  const std::vector<ObservableState> states{
      TEST_STATE_1,
      TEST_STATE_2,
      TARGET_STATE};

  // TEST
  const std::optional<double> min_distance_with_state =
      min_distance(TARGET_STATE, states);
  const std::optional<double> min_distance_without_state =
      min_distance(TARGET_STATE.id, states);

  // VERIFY
  EXPECT_TRUE(min_distance_with_state.has_value());
  EXPECT_TRUE(min_distance_without_state.has_value());

  // Optional values checked above
  // NOLINTBEGIN(bugprone-unchecked-optional-access)
  EXPECT_EQ(min_distance_with_state.value(), 1.0);
  EXPECT_EQ(min_distance_without_state.value(), 1.0);
  // NOLINTEND(bugprone-unchecked-optional-access)
}

TEST(MinDistanceTests, BadTime) {
  // SETUPz
  ObservableState bad_state{TEST_STATE_1};
  bad_state.time_of_validity = TIME + std::chrono::nanoseconds(1);
  const std::vector<ObservableState> states{
      TEST_STATE_1,
      bad_state,
      TARGET_STATE};

  // TEST + VERIFY
  EXPECT_THROW(min_distance(TARGET_STATE, states), AssertException);
  EXPECT_THROW(min_distance(TARGET_STATE.id, states), AssertException);
}

TEST(MinDistanceTests, BadFrame) {
  // SETUP
  ObservableState bad_state{TEST_STATE_1};
  bad_state.state.set_ref_from_body(
      FSE3(SE3::identity(), TARGET_FRAME, REF_FRAME));
  const std::vector<ObservableState> states{
      TEST_STATE_1,
      bad_state,
      TARGET_STATE};

  // TEST + VERIFY
  EXPECT_THROW(min_distance(TARGET_STATE, states), AssertException);
  EXPECT_THROW(min_distance(TARGET_STATE.id, states), AssertException);
}

TEST(MinDistanceTests, OnlyUnspawned) {
  // SETUP
  ObservableState unspawned_state_1{TEST_STATE_1};
  unspawned_state_1.is_spawned = false;
  ObservableState unspawned_state_2{TEST_STATE_2};
  unspawned_state_2.is_spawned = false;
  const std::vector<ObservableState> states{
      unspawned_state_1,
      unspawned_state_2,
      TARGET_STATE};

  // TEST
  const std::optional<double> d_with_state = min_distance(TARGET_STATE, states);
  const std::optional<double> d_without_state =
      min_distance(TARGET_STATE.id, states);

  // VERIFY
  EXPECT_EQ(d_with_state, std::nullopt);
  EXPECT_EQ(d_without_state, std::nullopt);
}

TEST(MinDistanceTests, IgnoresUnspawned) {
  // SETUP
  ObservableState unspawned_state{TEST_STATE_1};
  unspawned_state.is_spawned = false;
  const std::vector<ObservableState> states{
      unspawned_state,
      TEST_STATE_2,
      TARGET_STATE};

  // TEST
  const std::optional<double> d_with_state = min_distance(TARGET_STATE, states);
  const std::optional<double> d_without_state =
      min_distance(TARGET_STATE.id, states);

  // VERIFY
  EXPECT_TRUE(d_with_state.has_value());
  EXPECT_TRUE(d_without_state.has_value());

  EXPECT_EQ(d_with_state, 2.0);
  EXPECT_EQ(d_without_state, 2.0);
}

TEST(MinDistanceTests, TargetUnspawned) {
  // SETUP
  ObservableState unspawned_target{TARGET_STATE};
  unspawned_target.is_spawned = false;
  const std::vector<ObservableState> states{
      unspawned_target,
      TEST_STATE_1,
      TEST_STATE_2};

  // TEST + VERIFY
  EXPECT_THROW(min_distance(unspawned_target, states), AssertException);
  EXPECT_THROW(min_distance(unspawned_target.id, states), AssertException);
}
}  // namespace resim::metrics
