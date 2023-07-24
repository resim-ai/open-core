
#include "resim/actor/actor_unit.hh"

#include <fmt/core.h>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <variant>

#include "resim/actor/actor.hh"
#include "resim/actor/actor_id.hh"
#include "resim/actor/state/observable_state.hh"
#include "resim/actor/state/proto/observable_state.pb.h"
#include "resim/actor/state/proto/observable_state_to_proto.hh"
#include "resim/actor/test_actor.hh"
#include "resim/assert/assert.hh"
#include "resim/simulator/executor_builder.hh"
#include "resim/simulator/standard_frames.hh"
#include "resim/simulator/standard_topics.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/liegroup_test_helpers.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/testing/mock_logger.hh"

namespace resim::actor {

namespace {
using testing::MockLogger;
using Frame = transforms::Frame<transforms::SE3::DIMS>;

// Helper to verify that all of the test states are published into the
// channel_to_message_map with the given timestamp.
void expect_states_logged(
    const std::vector<state::ObservableState> &states,
    const MockLogger::ChannelToMessageMap &channel_to_message_map,
    const time::Timestamp time) {
  constexpr auto STATES_CHANNEL_NAME = "/actor_states";

  ASSERT_TRUE(channel_to_message_map.contains(STATES_CHANNEL_NAME));
  ASSERT_EQ(channel_to_message_map.at(STATES_CHANNEL_NAME).size(), 1U);
  const MockLogger::TimedMessage message{
      channel_to_message_map.at(STATES_CHANNEL_NAME).front()};
  EXPECT_EQ(message.time, time);

  state::proto::ObservableStates states_msg;
  pack(states, &states_msg);
  EXPECT_EQ(message.message, states_msg.SerializeAsString());
}

// GTEST macros end up causing clang-tidy to overestimate the complexity
// NOLINTBEGIN(readability-function-cognitive-complexity)
void test_actor_unit_end_to_end(
    std::vector<state::ObservableState> &states,
    const time::Timestamp &start_time) {
  MockLogger::ChannelToMessageMap channel_to_message_map;
  std::shared_ptr<MockLogger> logger =
      std::make_shared<MockLogger>(channel_to_message_map);

  std::vector<state::ObservableState> expected_states{};

  std::size_t simulated_forward = 0;
  constexpr time::Duration DELTA_TIME{std::chrono::nanoseconds(1)};

  simulator::ExecutorBuilder executor_builder;
  executor_builder.add_independent_task<time::Timestamp>(
      "publish_time",
      simulator::TIME_TOPIC,
      [start_time, &DELTA_TIME]() { return start_time + DELTA_TIME; });

  std::unique_ptr<ActorLoggerUnit> actor_logger_unit =
      std::make_unique<ActorLoggerUnit>(logger, InOut{executor_builder});

  std::vector<std::unique_ptr<ActorUnit>> units{};
  for (auto &state : states) {
    std::unique_ptr<TestActor> actor = std::make_unique<TestActor>(state.id);
    // actor->set_simulate_forward([
    actor->set_simulate_forward([actor = actor.get(),
                                 &state = state,
                                 &simulated_forward](const time::Timestamp t) {
      state.time_of_validity = t;
      actor->set_state(state);
      ++simulated_forward;
    });

    units.emplace_back(std::make_unique<ActorUnit>(
        logger,
        std::move(actor),
        InOut{executor_builder}));

    state::ObservableState expected_state{state};
    expected_state.time_of_validity = start_time + DELTA_TIME;
    expected_states.push_back(expected_state);
  }

  // Add tasks to check logs and publishing working correctly

  bool states_checked = false;
  executor_builder.add_task<state::ObservableState>(
      "check_states",
      simulator::ACTOR_STATES_TOPIC,
      simulator::NULL_TOPIC,
      [&states_checked, &expected_states, &simulated_forward](
          const std::vector<state::ObservableState> &states) {
        EXPECT_TRUE(simulated_forward);

        EXPECT_EQ(expected_states.size(), states.size());
        for (int i = 0; i < states.size(); ++i) {
          EXPECT_EQ(states[i].id, expected_states[i].id);
          EXPECT_EQ(
              states[i].time_of_validity,
              expected_states[i].time_of_validity);
          EXPECT_EQ(states[i].is_spawned, expected_states[i].is_spawned);
          EXPECT_TRUE(states[i].state.ref_from_body().is_approx(
              expected_states[i].state.ref_from_body()));
        }
        states_checked = true;
      });

  auto executor = executor_builder.build();

  // ACTION
  executor->run_step();

  EXPECT_TRUE(states_checked);
  EXPECT_EQ(simulated_forward, units.size());
  EXPECT_EQ(units.size(), states.size());
  expect_states_logged(
      expected_states,
      channel_to_message_map,
      start_time + DELTA_TIME);
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace

TEST(ActorUnitTest, TestActorUnitsEndToEnd) {
  constexpr int NUM_TESTS = 100;
  constexpr time::Timestamp TIME{time::Timestamp{} + std::chrono::seconds(1)};
  for (int i = 0; i < NUM_TESTS; ++i) {
    auto states = get_test_actor_states(TIME);
    test_actor_unit_end_to_end(states, TIME);
  }
}

TEST(ActorUnitTest, TestActorUnitBadStateTime) {
  const ActorId id{ActorId::new_uuid()};
  std::unique_ptr<TestActor> actor{std::make_unique<TestActor>(id)};
  MockLogger::ChannelToMessageMap channel_to_message_map;
  std::shared_ptr<MockLogger> logger =
      std::make_shared<MockLogger>(channel_to_message_map);

  actor->set_simulate_forward(
      [actor = actor.get()](const time::Timestamp time) {
        const state::ObservableState state{
            .id = actor->id(),
            .is_spawned = true,
            // Put in an incorrect time for this state:
            .time_of_validity = time + std::chrono::nanoseconds(1),
            .state = make_default_actor_state(),
        };
        actor->set_state(state);
      });

  simulator::ExecutorBuilder executor_builder;
  ActorUnit unit{std::move(logger), std::move(actor), InOut{executor_builder}};

  constexpr time::Timestamp TIME{std::chrono::seconds(1)};
  executor_builder.add_independent_task<time::Timestamp>(
      "publish_time",
      simulator::TIME_TOPIC,
      [TIME]() { return TIME; });

  auto executor = executor_builder.build();

  // ACTION / VERIFICATION
  EXPECT_THROW(executor->run_step(), AssertException);
}

TEST(ActorUnitTest, TestActorUnitBadStateId) {
  const ActorId id{ActorId::new_uuid()};
  std::unique_ptr<TestActor> actor{std::make_unique<TestActor>(id)};
  MockLogger::ChannelToMessageMap channel_to_message_map;
  std::shared_ptr<MockLogger> logger =
      std::make_shared<MockLogger>(channel_to_message_map);

  actor->set_simulate_forward(
      [actor = actor.get()](const time::Timestamp time) {
        const state::ObservableState state{
            .id = ActorId::new_uuid(),  // Wrong ID
            .is_spawned = true,
            .time_of_validity = time,
            .state = make_default_actor_state(),
        };
        actor->set_state(state);
      });

  simulator::ExecutorBuilder executor_builder;
  ActorUnit unit{std::move(logger), std::move(actor), InOut{executor_builder}};

  constexpr time::Timestamp TIME{std::chrono::seconds(1)};
  executor_builder.add_independent_task<time::Timestamp>(
      "publish_time",
      simulator::TIME_TOPIC,
      [TIME]() { return TIME; });

  auto executor = executor_builder.build();

  // ACTION / VERIFICATION
  EXPECT_THROW(executor->run_step(), AssertException);
}

TEST(ActorLoggerUnitTest, TestLogActorStates) {
  // SETUP
  MockLogger::ChannelToMessageMap channel_to_message_map;
  std::shared_ptr<MockLogger> logger =
      std::make_shared<MockLogger>(channel_to_message_map);
  constexpr time::Timestamp TIME{time::Timestamp{} + std::chrono::seconds(1)};

  simulator::ExecutorBuilder executor_builder;
  ActorLoggerUnit unit{std::move(logger), InOut{executor_builder}};

  std::vector<actor::state::ObservableState> test_states{
      get_test_actor_states(TIME)};
  for (const auto &state : test_states) {
    executor_builder.add_task<time::Timestamp, actor::state::ObservableState>(
        "publish_state",
        simulator::TIME_TOPIC,
        simulator::ACTOR_STATES_TOPIC,
        [&](time::Timestamp) { return state; });
  }

  // We need this to kick off the step:
  executor_builder.add_independent_task<time::Timestamp>(
      "publish_time",
      simulator::TIME_TOPIC,
      [&]() { return TIME; });

  std::unique_ptr<simulator::StepExecutor> executor{executor_builder.build()};

  // ACTION
  executor->run_step();

  // VERIFICATION
  expect_states_logged(test_states, channel_to_message_map, TIME);
}

}  // namespace resim::actor
