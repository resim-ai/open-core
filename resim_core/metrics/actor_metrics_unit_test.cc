#include "resim_core/metrics/actor_metrics_unit.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/actor/state/rigid_body_state.hh"
#include "resim_core/actor/test_actor.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/metrics/proto/simple_metric.pb.h"
#include "resim_core/metrics/simple_metric.hh"
#include "resim_core/simulator/executor_builder.hh"
#include "resim_core/simulator/standard_topics.hh"
#include "resim_core/simulator/step_executor.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/inout.hh"
#include "resim_core/utils/mcap_logger.hh"
#include "resim_core/utils/testing/mock_logger.hh"
#include "resim_core/utils/uuid.hh"

namespace resim::metrics {

namespace {

const std::string MIN_DISTANCE_TOPIC_NAME = "metric_min_distance";

const resim::UUID EGO_UUID = UUID::new_uuid();
constexpr std::size_t EGO_INDEX = 0;

using Frame = transforms::Frame<transforms::FSE3::DIMS>;
using actor::state::ObservableState;
using testing::MockLogger;
}  // namespace

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(ActorMetricsUnitTest, TestLogMinDistanceMetric) {
  // SETUP
  simulator::ExecutorBuilder executor_builder;

  MockLogger::ChannelToMessageMap channel_to_message_map;

  constexpr time::Timestamp TIME{time::Timestamp{} + std::chrono::seconds(1)};

  std::vector<ObservableState> test_states{actor::get_test_actor_states(TIME)};
  test_states[EGO_INDEX].id = EGO_UUID;

  for (const auto &state : test_states) {
    executor_builder.add_independent_task<ObservableState>(
        "publish_state",
        simulator::ACTOR_STATES_TOPIC,
        [&]() { return state; });
  }

  const std::unique_ptr<ActorMetricsUnit> unit =
      std::make_unique<ActorMetricsUnit>(
          std::make_unique<MockLogger>(channel_to_message_map),
          InOut{executor_builder},
          EGO_UUID);

  std::unique_ptr<simulator::StepExecutor> executor{executor_builder.build()};

  // ACTION
  executor->run_step();

  // VERIFICATION
  EXPECT_TRUE(channel_to_message_map.contains(MIN_DISTANCE_TOPIC_NAME));
  ASSERT_EQ(channel_to_message_map.at(MIN_DISTANCE_TOPIC_NAME).size(), 1U);

  const MockLogger::TimedMessage message{
      channel_to_message_map.at(MIN_DISTANCE_TOPIC_NAME).front()};
  EXPECT_EQ(message.time, TIME);

  metrics::proto::SimpleMetric metric_msg;
  metric_msg.ParseFromString(message.message);

  EXPECT_EQ(metric_msg.name(), MIN_DISTANCE_TOPIC_NAME);
  EXPECT_TRUE(metric_msg.has_metric_value());
  EXPECT_GT(metric_msg.metric_value(), 0.0);
  EXPECT_EQ(
      time::Timestamp(time::from_seconds_and_nanos(
          {metric_msg.time().seconds(), metric_msg.time().nanos()})),
      TIME);
}
// NOLINTEND(readability-function-cognitive-complexity)

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(ActorMetricsUnitTest, TestLogMinDistanceMetricEmpty) {
  // SETUP
  simulator::ExecutorBuilder executor_builder;

  MockLogger::ChannelToMessageMap channel_to_message_map;

  const ActorMetricsUnit unit{
      std::make_unique<MockLogger>(channel_to_message_map),
      InOut{executor_builder},
      EGO_UUID};

  constexpr time::Timestamp TIME{time::Timestamp{} + std::chrono::seconds(1)};
  const Frame EGO_FRAME = Frame::new_frame();

  std::vector<actor::state::ObservableState> test_states{
      actor::state::ObservableState{
          .id = EGO_UUID,
          .is_spawned = true,
          .time_of_validity = TIME,
          .state =
              actor::state::RigidBodyState<transforms::FSE3>{
                  transforms::FSE3::identity(
                      simulator::SCENE_FRAME,
                      EGO_FRAME)},
      }};
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
  EXPECT_TRUE(channel_to_message_map.contains(MIN_DISTANCE_TOPIC_NAME));
  ASSERT_EQ(channel_to_message_map.at(MIN_DISTANCE_TOPIC_NAME).size(), 1U);

  const MockLogger::TimedMessage message{
      channel_to_message_map.at(MIN_DISTANCE_TOPIC_NAME).front()};
  EXPECT_EQ(message.time, TIME);

  metrics::proto::SimpleMetric metric_msg;
  metric_msg.ParseFromString(message.message);

  EXPECT_EQ(metric_msg.name(), MIN_DISTANCE_TOPIC_NAME);
  EXPECT_FALSE(metric_msg.has_metric_value());
  EXPECT_EQ(
      time::Timestamp(time::from_seconds_and_nanos(
          {metric_msg.time().seconds(), metric_msg.time().nanos()})),
      TIME);
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim::metrics
