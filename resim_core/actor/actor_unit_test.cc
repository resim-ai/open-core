
#include "resim_core/actor/actor_unit.hh"

#include <fmt/core.h>
#include <foxglove/FrameTransform.pb.h>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <variant>

#include "resim_core/actor/actor.hh"
#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/geometry.hh"
#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/actor/test_actor.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/simulator/executor_builder.hh"
#include "resim_core/simulator/standard_frames.hh"
#include "resim_core/simulator/standard_topics.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/inout.hh"
#include "resim_core/utils/testing/mock_logger.hh"
#include "resim_core/visualization/foxglove/actor_geometry_to_foxglove.hh"
#include "resim_core/visualization/foxglove/frame_transform_to_foxglove.hh"

namespace resim::actor {

namespace {
using testing::MockLogger;
using Frame = transforms::Frame<transforms::FSE3::DIMS>;

// Helper to verify that all of the test states are published into the
// channel_to_message_map with the given timestamp.
void expect_states_logged(
    const std::vector<state::ObservableState> &states,
    const MockLogger::ChannelToMessageMap &channel_to_message_map,
    const time::Timestamp time) {
  for (const auto &state : states) {
    const std::string channel_name{fmt::format(
        "/transforms/{}_from_{}",
        simulator::SCENE_FRAME_NAME,
        state.state.ref_from_body().from().id().to_string())};
    EXPECT_TRUE(channel_to_message_map.contains(channel_name));
    ASSERT_EQ(channel_to_message_map.at(channel_name).size(), 1U);

    const MockLogger::TimedMessage message{
        channel_to_message_map.at(channel_name).front()};
    EXPECT_EQ(message.time, time);
    foxglove::FrameTransform transform;

    visualization::foxglove::pack_into_foxglove(
        state.state.ref_from_body(),
        time,
        &transform,
        std::string(simulator::SCENE_FRAME_NAME));
    EXPECT_EQ(message.message, transform.SerializeAsString());
  }
}

// Helper for comparing a SceneEntityDeletion with a Clear geometry below
void expect_deletion_matches_geometry(
    const ::foxglove::SceneEntityDeletion &deletion,
    const Geometry &geometry) {
  ASSERT_TRUE(
      std::holds_alternative<Geometry::Clear>(geometry.visible_geometry));
  const auto &time_msg = deletion.timestamp();
  EXPECT_EQ(
      time_msg.seconds() * std::nano::den + time_msg.nanos(),
      geometry.time_of_validity.time_since_epoch().count());
  EXPECT_EQ(deletion.id(), geometry.frame.id().to_string());
}

// Helper to verify that all of the test geometries are published into the
// channel_to_message_map with the given timestamp.
void expect_geometries_logged(
    const std::vector<Geometry> &geometries,
    const MockLogger::ChannelToMessageMap &channel_to_message_map,
    const time::Timestamp time) {
  constexpr auto GEOMETRIES_TOPIC = "/geometries";
  ASSERT_TRUE(channel_to_message_map.contains(GEOMETRIES_TOPIC));
  ASSERT_EQ(channel_to_message_map.at(GEOMETRIES_TOPIC).size(), 1U);

  const MockLogger::TimedMessage message{
      channel_to_message_map.at(GEOMETRIES_TOPIC).front()};
  EXPECT_EQ(message.time, time);

  ::foxglove::SceneUpdate scene_update;
  ASSERT_TRUE(scene_update.ParseFromString(message.message));
  // All geometries are Clear operations
  ASSERT_EQ(scene_update.deletions_size(), geometries.size());
  for (int ii = 0; ii < scene_update.deletions_size(); ++ii) {
    expect_deletion_matches_geometry(
        scene_update.deletions(ii),
        geometries.at(ii));
  }
}

// GTEST macros end up causing clang-tidy to overestimate the complexity
// NOLINTBEGIN(readability-function-cognitive-complexity)
void test_actor_unit_end_to_end(
    std::vector<std::pair<state::ObservableState, Geometry>>
        &states_and_geometries,
    const time::Timestamp &start_time) {
  MockLogger::ChannelToMessageMap channel_to_message_map;
  std::shared_ptr<MockLogger> logger =
      std::make_shared<MockLogger>(channel_to_message_map);

  std::vector<state::ObservableState> expected_states{};
  std::vector<Geometry> expected_geometries{};

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
  for (auto &[state, geometry] : states_and_geometries) {
    std::unique_ptr<TestActor> actor = std::make_unique<TestActor>(state.id);
    // actor->set_simulate_forward([
    actor->set_simulate_forward([actor = actor.get(),
                                 &state = state,
                                 &geometry = geometry,
                                 &simulated_forward](const time::Timestamp t) {
      state.time_of_validity = t;
      geometry.time_of_validity = t;
      actor->set_state(state);
      actor->set_geometry(geometry);
      ++simulated_forward;
    });

    units.emplace_back(std::make_unique<ActorUnit>(
        logger,
        std::move(actor),
        InOut{executor_builder}));

    state::ObservableState expected_state{state};
    expected_state.time_of_validity = start_time + DELTA_TIME;
    Geometry expected_geometry{geometry};
    expected_geometry.time_of_validity = start_time + DELTA_TIME;

    expected_states.push_back(expected_state);
    expected_geometries.push_back(expected_geometry);
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

  bool geometries_checked = false;
  executor_builder.add_task<Geometry>(
      "check_geometries",
      simulator::ACTOR_GEOMETRIES_TOPIC,
      simulator::NULL_TOPIC,
      [&expected_geometries, &geometries_checked, &simulated_forward](
          const std::vector<Geometry> &geometries) {
        EXPECT_TRUE(simulated_forward);

        EXPECT_EQ(expected_geometries.size(), geometries.size());
        for (int i = 0; i < geometries.size(); ++i) {
          EXPECT_TRUE(std::holds_alternative<Geometry::Clear>(
              geometries[i].visible_geometry));
          EXPECT_EQ(geometries[i].frame, expected_geometries[i].frame);
          EXPECT_EQ(
              geometries[i].time_of_validity,
              expected_geometries[i].time_of_validity);
        }
        geometries_checked = true;
      });
  auto executor = executor_builder.build();

  // ACTION
  executor->run_step();

  EXPECT_TRUE(states_checked);
  EXPECT_TRUE(geometries_checked);
  EXPECT_EQ(simulated_forward, units.size());
  EXPECT_EQ(units.size(), states_and_geometries.size());
  expect_states_logged(
      expected_states,
      channel_to_message_map,
      start_time + DELTA_TIME);
  expect_geometries_logged(
      expected_geometries,
      channel_to_message_map,
      start_time + DELTA_TIME);
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace

TEST(ActorUnitTest, TestActorUnitsEndToEnd) {
  constexpr int NUM_TESTS = 100;
  constexpr time::Timestamp TIME{time::Timestamp{} + std::chrono::seconds(1)};
  for (int i = 0; i < NUM_TESTS; ++i) {
    auto states_and_geometries = get_test_actor_components(TIME);
    test_actor_unit_end_to_end(states_and_geometries, TIME);
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
    executor_builder.add_independent_task<actor::state::ObservableState>(
        "publish_state",
        simulator::ACTOR_STATES_TOPIC,
        [&]() { return state; });
  }

  std::unique_ptr<simulator::StepExecutor> executor{executor_builder.build()};

  // ACTION
  executor->run_step();

  // VERIFICATION
  expect_states_logged(test_states, channel_to_message_map, TIME);
}

TEST(ActorLoggerUnitTest, TestLogActorGeometries) {
  // SETUP
  MockLogger::ChannelToMessageMap channel_to_message_map;
  std::shared_ptr<MockLogger> logger =
      std::make_shared<MockLogger>(channel_to_message_map);

  simulator::ExecutorBuilder executor_builder;
  ActorLoggerUnit unit{std::move(logger), InOut{executor_builder}};
  constexpr time::Timestamp TIME{time::Timestamp{} + std::chrono::seconds(1)};

  std::vector<actor::Geometry> test_geometries{get_test_actor_geometries(TIME)};

  for (const auto &geometry : test_geometries) {
    executor_builder.add_independent_task<actor::Geometry>(
        "publish_geometry",
        simulator::ACTOR_GEOMETRIES_TOPIC,
        [&]() { return geometry; });
  }

  std::unique_ptr<simulator::StepExecutor> executor{executor_builder.build()};

  // ACTION
  executor->run_step();

  // VERIFICATION
  expect_geometries_logged(test_geometries, channel_to_message_map, TIME);
}

TEST(ActorLoggerUnitTest, TestLogActorGeometriesInconsistentTimes) {
  // SETUP
  MockLogger::ChannelToMessageMap channel_to_message_map;
  std::shared_ptr<MockLogger> logger =
      std::make_shared<MockLogger>(channel_to_message_map);

  simulator::ExecutorBuilder executor_builder;
  ActorLoggerUnit unit{std::move(logger), InOut{executor_builder}};
  constexpr time::Timestamp TIME{time::Timestamp{} + std::chrono::seconds(1)};

  constexpr bool INCONSISTENT_TIMES = true;
  std::vector<actor::Geometry> test_geometries{
      get_test_actor_geometries(TIME, INCONSISTENT_TIMES)};

  for (const auto &geometry : test_geometries) {
    executor_builder.add_independent_task<actor::Geometry>(
        "publish_geometry",
        simulator::ACTOR_GEOMETRIES_TOPIC,
        [&]() { return geometry; });
  }

  std::unique_ptr<simulator::StepExecutor> executor{executor_builder.build()};

  // ACTION / VERIFICATION
  EXPECT_THROW(executor->run_step(), AssertException);
}

}  // namespace resim::actor
