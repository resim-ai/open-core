
#include "resim/simulator/time_lord.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "resim/assert/assert.hh"
#include "resim/simulator/executor_builder.hh"
#include "resim/simulator/standard_topics.hh"
#include "resim/simulator/step_executor.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/inout.hh"

namespace resim::simulator {

TEST(TimeLordTest, TestTimeLord) {
  // SETUP
  ExecutorBuilder executor_builder;
  time::Timestamp expected_time;
  bool received_time = false;
  executor_builder.add_task<time::Timestamp>(
      "expect_times_match",
      TIME_TOPIC,
      NULL_TOPIC,
      [&](const time::Timestamp current_time) {
        EXPECT_EQ(current_time, expected_time);
        received_time = true;
      });

  // ACTION
  TimeLord time_lord{nullptr, InOut{executor_builder}};

  const std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  constexpr time::Duration DT{std::chrono::seconds(1)};
  constexpr int NUM_STEPS = 100;

  time::Timestamp schedule_time;
  for (int ii = 0; ii < NUM_STEPS; ++ii) {
    time_lord.schedule_update(TimeLordUpdate{
        schedule_time,
        TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
        TimeLordUpdate::UpdateType::PRESCHEDULED});
    schedule_time += DT;
  }

  for (int ii = 0; ii < NUM_STEPS; ++ii) {
    EXPECT_FALSE(time_lord.ready_to_terminate());
    received_time = false;

    executor->run_step();
    expected_time += DT;

    // VERIFICATION
    EXPECT_TRUE(received_time);
  }

  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(TimeLordDeathTest, TestTimeLordBadDT) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};

  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  executor->run_step();

  // ACTION / VERIFICATION
  EXPECT_THROW(
      time_lord.schedule_update(TimeLordUpdate{
          time::Timestamp(std::chrono::seconds(-1)),
          TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
          TimeLordUpdate::UpdateType::PRESCHEDULED}),
      AssertException);
}

TEST(TimeLordTest, TimeLordScheduleAfterStepTest) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};
  bool received_time = false;
  time::Timestamp expected_time;

  executor_builder.add_task<time::Timestamp>(
      "expect_times_match",
      TIME_TOPIC,
      NULL_TOPIC,
      [&](const time::Timestamp current_time) {
        EXPECT_EQ(current_time, expected_time);
        received_time = true;
      });

  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(1)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});

  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(3)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // ACTION / VERIFICATION

  received_time = false;
  expected_time = time::Timestamp(std::chrono::seconds(1));
  executor->run_step();
  EXPECT_TRUE(received_time);

  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(2)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});

  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(4)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});

  received_time = false;
  expected_time = time::Timestamp(std::chrono::seconds(2));
  executor->run_step();
  EXPECT_TRUE(received_time);

  received_time = false;
  expected_time = time::Timestamp(std::chrono::seconds(3));
  executor->run_step();
  EXPECT_TRUE(received_time);

  received_time = false;
  expected_time = time::Timestamp(std::chrono::seconds(4));
  executor->run_step();
  EXPECT_TRUE(received_time);

  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(TimeLordTest, TimeLordReadyToTerminate) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};

  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(1)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});
  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(2)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE_THEN_TERMINATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});
  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(3)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  EXPECT_FALSE(time_lord.ready_to_terminate());
  executor->run_step();
  EXPECT_FALSE(time_lord.ready_to_terminate());
  executor->run_step();
  EXPECT_TRUE(time_lord.ready_to_terminate());

  EXPECT_THROW(executor->run_step(), AssertException);
}

TEST(TimeLordTest, TimeLordTaskSchedulesUpdate) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};

  bool received_time = false;
  time::Timestamp expected_time;

  executor_builder.add_task<time::Timestamp>(
      "expect_times_match",
      TIME_TOPIC,
      NULL_TOPIC,
      [&](const time::Timestamp current_time) {
        EXPECT_EQ(current_time, expected_time);
        received_time = true;
      });
  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});
  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(1)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});
  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(3)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});

  executor_builder.add_task<time::Timestamp, std::vector<TimeLordUpdate>>(
      "schedule_intermediate_step",
      TIME_TOPIC,
      SCHEDULE_TIMELORD_TOPIC,
      [&](const time::Timestamp current_time) {
        std::vector<TimeLordUpdate> updates;
        if (current_time == time::Timestamp(std::chrono::seconds(1))) {
          updates.emplace_back(
              time::Timestamp(std::chrono::seconds(2)),
              TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
              TimeLordUpdate::UpdateType::UNIT_SCHEDULED);
        }
        return updates;
      });

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};
  for (int ii = 0; ii < 4; ++ii) {
    EXPECT_FALSE(time_lord.ready_to_terminate());
    received_time = false;
    expected_time = time::Timestamp(std::chrono::seconds(ii));

    executor->run_step();

    // VERIFICATION
    EXPECT_TRUE(received_time);
  }

  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(TimeLordTest, TerminateOnSameStep) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};

  bool received_time = false;
  time::Timestamp expected_time{std::chrono::seconds(0)};

  executor_builder.add_task<time::Timestamp>(
      "expect_times_match",
      TIME_TOPIC,
      NULL_TOPIC,
      [&](const time::Timestamp current_time) {
        EXPECT_EQ(current_time, expected_time);
        received_time = true;
      });
  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});
  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(1)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});

  executor_builder.add_task<time::Timestamp, std::vector<TimeLordUpdate>>(
      "schedule_early_terminate",
      TIME_TOPIC,
      SCHEDULE_TIMELORD_TOPIC,
      [&](const time::Timestamp current_time) {
        std::vector<TimeLordUpdate> updates{TimeLordUpdate(
            time::Timestamp(current_time),
            TimeLordUpdate::UpdateBehaviour::FULL_UPDATE_THEN_TERMINATE,
            TimeLordUpdate::UpdateType::UNIT_SCHEDULED)};
        return updates;
      });

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};
  EXPECT_FALSE(time_lord.ready_to_terminate());

  executor->run_step();

  // VERIFICATION
  EXPECT_TRUE(received_time);
  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(TimeLordTest, RepeatTimesRightOrder) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};

  bool received_time = false;
  time::Timestamp expected_time{std::chrono::seconds(0)};

  executor_builder.add_task<time::Timestamp>(
      "expect_times_match",
      TIME_TOPIC,
      NULL_TOPIC,
      [&](const time::Timestamp current_time) {
        EXPECT_EQ(current_time, expected_time);
        received_time = true;
      });
  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});
  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE_THEN_TERMINATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});
  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(1)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};
  EXPECT_FALSE(time_lord.ready_to_terminate());

  executor->run_step();

  // VERIFICATION
  EXPECT_TRUE(received_time);
  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(TimeLordTest, SkipUpdateScheduledOnSameStep) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};

  bool received_time = false;
  time::Timestamp expected_time{std::chrono::seconds(0)};

  executor_builder.add_task<time::Timestamp>(
      "expect_times_match",
      TIME_TOPIC,
      NULL_TOPIC,
      [&](const time::Timestamp current_time) {
        EXPECT_EQ(current_time, expected_time);
        received_time = true;
      });
  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});
  time_lord.schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(1)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});

  executor_builder.add_task<time::Timestamp, std::vector<TimeLordUpdate>>(
      "schedule_early_terminate",
      TIME_TOPIC,
      SCHEDULE_TIMELORD_TOPIC,
      [&](const time::Timestamp current_time) {
        std::vector<TimeLordUpdate> updates{TimeLordUpdate(
            time::Timestamp(current_time),
            TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
            TimeLordUpdate::UpdateType::UNIT_SCHEDULED)};
        return updates;
      });

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};
  EXPECT_FALSE(time_lord.ready_to_terminate());

  // VERIFICATION
  received_time = false;
  expected_time = time::Timestamp(std::chrono::seconds(0));
  executor->run_step();

  EXPECT_TRUE(received_time);
  EXPECT_FALSE(time_lord.ready_to_terminate());

  received_time = false;
  expected_time = time::Timestamp(std::chrono::seconds(1));
  executor->run_step();
  EXPECT_TRUE(received_time);
  EXPECT_TRUE(time_lord.ready_to_terminate());
}

}  // namespace resim::simulator
