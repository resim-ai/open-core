
#include "resim_core/simulator/time_lord.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "resim_core/assert/assert.hh"
#include "resim_core/simulator/executor_builder.hh"
#include "resim_core/simulator/standard_topics.hh"
#include "resim_core/simulator/step_executor.hh"
#include "resim_core/utils/inout.hh"

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
  for (int ii = 0; ii < NUM_STEPS; ++ii) {
    received_time = false;
    time_lord.increment_time(DT);
    expected_time += DT;

    executor->run_step();
    // VERIFICATION
    EXPECT_TRUE(received_time);
  }
}

TEST(TimeLordDeathTest, TestTimeLordBadDT) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};
  constexpr time::Duration DT{-std::chrono::seconds(1)};

  // ACTION / VERIFICATION
  EXPECT_THROW(time_lord.increment_time(DT), AssertException);
}

}  // namespace resim::simulator
