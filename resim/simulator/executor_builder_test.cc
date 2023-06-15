
#include "resim/simulator/executor_builder.hh"

#include <gtest/gtest.h>

#include <memory>
#include <numeric>

#include "resim/assert/assert.hh"
#include "resim/simulator/step_executor.hh"
#include "resim/utils/inout.hh"

namespace resim::simulator {

namespace {

// A simple helper that checks that a given task_ran flag is set when
// the executor is run. This is used frequently since we're often
// trying to verify that a specific task has been successfully run
// because our dependencies and provisions are set up correctly by the
// ExecutorBuilder.
// @param[inout] executor_builder - The builder we're testing.
// @param[inout] task_ran - A flag that should be set to true when and
//                          only when the executor resulting from
//                          executor_builder.build() is run.
void expect_task_runs_correctly(
    InOut<ExecutorBuilder> executor_builder,
    InOut<bool> task_ran) {
  EXPECT_FALSE(*task_ran);

  const std::unique_ptr<StepExecutor> executor{executor_builder->build()};

  EXPECT_FALSE(*task_ran);
  executor->run_step();
  EXPECT_TRUE(*task_ran);
}

// A null provision that no-one depends on.
constexpr auto NULL_PROVISION = "null";

}  // namespace

TEST(ExecutorBuilderTest, TestEmptyBuild) {
  ExecutorBuilder executor_builder;
  const std::unique_ptr<StepExecutor> executor{executor_builder.build()};
  executor->run_step();
}

TEST(ExecutorBuilderTest, TestAddIndependentTaskNoSend) {
  // SETUP
  ExecutorBuilder executor_builder;

  // ACTION
  bool task_ran = false;
  executor_builder.add_independent_task("test_task", "task_ran", [&task_ran]() {
    task_ran = true;
  });

  // VERIFICATION
  expect_task_runs_correctly(InOut{executor_builder}, InOut{task_ran});
}

TEST(ExecutorBuilderTest, TestAddIndependentTaskSend) {
  // SETUP
  ExecutorBuilder executor_builder;

  // ACTION
  bool task_ran = false;
  constexpr double TEST_VALUE = 2.;
  constexpr auto VALUE_KEY = "val_key";
  executor_builder
      .add_independent_task<double>(
          "predecessor_task",
          VALUE_KEY,
          [&]() {
            task_ran = true;
            return TEST_VALUE;
          })
      .add_task<double>(
          "test_task",
          VALUE_KEY,
          NULL_PROVISION,
          [&](const double value) { EXPECT_EQ(value, TEST_VALUE); });

  // VERIFICATION
  expect_task_runs_correctly(InOut{executor_builder}, InOut{task_ran});
}

TEST(ExecutorBuilderTest, TestAddTaskReceiveMultipleNoSend) {
  // SETUP
  ExecutorBuilder executor_builder;

  // ACTION
  bool task_ran = false;
  constexpr std::array<double, 3> TEST_VALUES = {2., 4., 6.};
  constexpr auto VALUE_KEY = "val_key";
  executor_builder
      .add_independent_task<double>(
          "predecessor_a",
          VALUE_KEY,
          [&]() { return TEST_VALUES.at(0); })
      .add_independent_task<double>(
          "predecessor_b",
          VALUE_KEY,
          [&]() { return TEST_VALUES.at(1); })
      .add_independent_task<double>(
          "predecessor_c",
          VALUE_KEY,
          [&]() { return TEST_VALUES.at(2); })
      .add_task<double>(
          "test_task",
          VALUE_KEY,
          NULL_PROVISION,
          [&](const std::vector<double> &vals) {
            task_ran = true;
            EXPECT_EQ(vals.size(), TEST_VALUES.size());
            for (const auto &val : TEST_VALUES) {
              EXPECT_NE(
                  std::find(vals.cbegin(), vals.cend(), val),
                  vals.cend());
            }
          });

  // VERIFICATION
  expect_task_runs_correctly(InOut{executor_builder}, InOut{task_ran});
}

TEST(ExecutorBuilderTest, TestAddTaskReceiveMultipleSend) {
  // SETUP
  ExecutorBuilder executor_builder;

  // ACTION
  bool task_ran = false;
  constexpr std::array<double, 3> TEST_VALUES = {2., 4., 6.};
  constexpr auto PREDECESSOR_KEY = "pred_key";
  constexpr auto SUCCESSOR_KEY = "succ_key";

  executor_builder
      .add_independent_task<double>(
          "predecessor_task_a",
          PREDECESSOR_KEY,
          [&]() { return TEST_VALUES.at(0); })
      .add_independent_task<double>(
          "predecessor_task_b",
          PREDECESSOR_KEY,
          [&]() { return TEST_VALUES.at(1); })
      .add_independent_task<double>(
          "predecessor_task_c",
          PREDECESSOR_KEY,
          [&]() { return TEST_VALUES.at(2); })
      .add_task<double, double>(
          "test_task",
          PREDECESSOR_KEY,
          SUCCESSOR_KEY,
          [&](const std::vector<double> &vals) {
            return std::accumulate(vals.cbegin(), vals.cend(), 0.);
          })
      .add_task<double>(
          "successor_task",
          SUCCESSOR_KEY,
          NULL_PROVISION,
          [&](const double val) {
            task_ran = true;
            EXPECT_EQ(
                std::accumulate(TEST_VALUES.cbegin(), TEST_VALUES.cend(), 0.),
                val);
          });

  // VERIFICATION
  expect_task_runs_correctly(InOut{executor_builder}, InOut{task_ran});
}

TEST(ExecutorBuilderTest, TestAddTaskReceiveSingleNoSend) {
  // SETUP
  ExecutorBuilder executor_builder;

  // ACTION
  bool task_ran = false;
  constexpr double TEST_VALUE = 2.;
  constexpr auto VALUE_KEY = "val_key";
  executor_builder
      .add_independent_task<double>(
          "predecessor",
          VALUE_KEY,
          [&]() { return TEST_VALUE; })
      .add_task<double>(
          "test_task",
          VALUE_KEY,
          NULL_PROVISION,
          [&](const double val) {
            task_ran = true;
            EXPECT_EQ(val, TEST_VALUE);
          });

  // VERIFICATION
  expect_task_runs_correctly(InOut{executor_builder}, InOut{task_ran});
}

TEST(ExecutorBuilderTest, TestAddTaskReceiveSingleSend) {
  // SETUP
  ExecutorBuilder executor_builder;

  // ACTION
  bool task_ran = false;
  constexpr double PREDECESSOR_VALUE = 2.;
  constexpr auto PREDECESSOR_KEY = "pred_key";
  constexpr double SUCCESSOR_VALUE = 3.;
  constexpr auto SUCCESSOR_KEY = "succ_key";
  executor_builder
      .add_independent_task<double>(
          "predecessor",
          PREDECESSOR_KEY,
          [&]() { return PREDECESSOR_VALUE; })
      .add_task<double, double>(
          "test_task",
          PREDECESSOR_KEY,
          SUCCESSOR_KEY,
          [&](const double val) {
            EXPECT_EQ(val, PREDECESSOR_VALUE);
            return SUCCESSOR_VALUE;
          })
      .add_task<double>(
          "successor",
          SUCCESSOR_KEY,
          NULL_PROVISION,
          [&](const double val) {
            task_ran = true;
            EXPECT_EQ(val, SUCCESSOR_VALUE);
          });

  // VERIFICATION
  expect_task_runs_correctly(InOut{executor_builder}, InOut{task_ran});
}

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(ExecutorBuilderDeathTest, TestBadAddTaskReceiveSingleNoSend) {
  // SETUP
  ExecutorBuilder executor_builder;

  // ACTION
  constexpr double TEST_VALUE = 2.;
  constexpr auto VALUE_KEY = "val_key";

  executor_builder
      .add_independent_task<double>(
          "predecessor",
          VALUE_KEY,
          [&]() { return TEST_VALUE; })
      .add_independent_task<double>(
          "predecessor",
          VALUE_KEY,
          [&]() { return TEST_VALUE; })
      .add_task<double>(
          "test_task",
          VALUE_KEY,
          NULL_PROVISION,
          [&](const double val) {});
  const std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // ACTION / VERIFICATION
  EXPECT_THROW(executor->run_step(), AssertException);
}
// NOLINTEND(readability-function-cognitive-complexity)

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(ExecutorBuilderDeathTest, TestBadAddTaskReceiveSingleSend) {
  // SETUP
  ExecutorBuilder executor_builder;

  // ACTION
  constexpr double TEST_VALUE = 2.;
  constexpr auto VALUE_KEY = "val_key";

  executor_builder
      .add_independent_task<double>(
          "predecessor",
          VALUE_KEY,
          [&]() { return TEST_VALUE; })
      .add_independent_task<double>(
          "predecessor",
          VALUE_KEY,
          [&]() { return TEST_VALUE; })
      .add_task<double, double>(
          "test_task",
          VALUE_KEY,
          NULL_PROVISION,
          [&](const double val) { return TEST_VALUE; });
  const std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // ACTION / VERIFICATION
  EXPECT_THROW(executor->run_step(), AssertException);
}
// NOLINTEND(readability-function-cognitive-complexity)

TEST(ExecutorBuilderTest, TestAddTaskNoReceiveNoSend) {
  // SETUP
  ExecutorBuilder executor_builder;

  // ACTION
  bool task_ran = false;
  constexpr auto KEY = "key";
  executor_builder.add_independent_task("predecessor", KEY, []() {})
      .add_task("test_task", KEY, NULL_PROVISION, [&]() { task_ran = true; });

  // VERIFICATION
  expect_task_runs_correctly(InOut{executor_builder}, InOut{task_ran});
}

TEST(ExecutorBuilderTest, TestAddTaskNoReceiveSend) {
  // SETUP
  ExecutorBuilder executor_builder;

  // ACTION
  bool task_ran = false;
  constexpr auto KEY = "key";
  constexpr double SUCCESSOR_VALUE = 2.;
  constexpr auto SUCCESSOR_KEY = "succ_key";
  executor_builder.add_independent_task("predecessor", KEY, [&]() {})
      .add_task<double>(
          "test_task",
          KEY,
          SUCCESSOR_KEY,
          [&]() { return SUCCESSOR_VALUE; })
      .add_task<double>(
          "successor",
          SUCCESSOR_KEY,
          NULL_PROVISION,
          [&](const double val) {
            task_ran = true;
            EXPECT_EQ(val, SUCCESSOR_VALUE);
          });

  // VERIFICATION
  expect_task_runs_correctly(InOut{executor_builder}, InOut{task_ran});
}

TEST(ExecutorBuilderTest, TestAddConditionalTaskTrue) {
  // SETUP
  ExecutorBuilder executor_builder;

  // ACTION
  bool task_ran = false;
  constexpr auto KEY = "key";
  constexpr auto SUCCESSOR_KEY = "succ_key";
  executor_builder.add_independent_task("predecessor", KEY, []() {})
      .add_conditional_task(
          "test_task",
          KEY,
          SUCCESSOR_KEY,
          []() { return true; })
      .add_task("successor", SUCCESSOR_KEY, NULL_PROVISION, [&]() {
        task_ran = true;
      });

  // VERIFICATION
  expect_task_runs_correctly(InOut{executor_builder}, InOut{task_ran});
}

TEST(ExecutorBuilderTest, TestAddConditionalTaskFalse) {
  // SETUP
  ExecutorBuilder executor_builder;

  // ACTION
  bool task_ran = false;
  constexpr auto KEY = "key";
  constexpr auto SUCCESSOR_KEY = "succ_key";
  executor_builder.add_independent_task("predecessor", KEY, []() {})
      .add_conditional_task(
          "test_task",
          KEY,
          SUCCESSOR_KEY,
          []() { return false; })
      .add_task("successor", SUCCESSOR_KEY, NULL_PROVISION, [&]() {
        task_ran = true;
      });

  // ACTION
  EXPECT_FALSE(task_ran);
  const std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // VERIFICATION
  EXPECT_FALSE(task_ran);
  executor->run_step();
  // We expect still false:
  EXPECT_FALSE(task_ran);
}

}  // namespace resim::simulator
