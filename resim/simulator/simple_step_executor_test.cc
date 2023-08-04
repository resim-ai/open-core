// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/simulator/simple_step_executor.hh"

#include <gtest/gtest.h>

#include <random>
#include <unordered_map>
#include <unordered_set>

#include "resim/utils/inout.hh"
#include "resim/utils/uuid.hh"

namespace resim::simulator {

namespace {

using Task = StepExecutor::Task;

// Generate a random boolean
template <typename RNG>
bool random_bool(RNG &&rng) {
  std::uniform_int_distribution<int> dist{0, 1};
  return dist(rng) == 0;
}

// Helper class to use for generating random task graphs and owning common data
// structures used by all of the tasks as we're testing.
class RandomTaskGraphMaker {
 public:
  // Construct this graph maker with 20 arbitrary checkpoints.
  RandomTaskGraphMaker() {
    constexpr std::size_t NUM_CHECKPOINTS = 20;
    checkpoints_.reserve(NUM_CHECKPOINTS);

    for (std::size_t ii = 0; ii < NUM_CHECKPOINTS; ++ii) {
      checkpoints_.push_back(UUID::new_uuid().to_string());
    }
  }

  // Make a random task graph between our checkpoints by adding tasks with 50
  // percent probability between each checkpoint and each checkpoint before it
  // in the list of checkpoints. This function resets the checkpoint_to_upstream
  // tasks map and executed task set since we're about to make a whole bunch of
  // new tasks.
  // @param[in] rng - The random number generator to use.
  // @param[in] run_count - A reference to a size_t to increment when each task
  //                         runs.
  template <typename RNG>
  std::vector<Task> build(RNG &&rng, InOut<std::size_t> run_count);

 private:
  using TaskId = int;
  using CheckpointToTaskMultiMap =
      std::unordered_multimap<std::string_view, TaskId>;
  using TaskSet = std::unordered_set<TaskId>;

  CheckpointToTaskMultiMap checkpoint_to_upstream_tasks_map_;
  TaskSet executed_tasks_;

  std::vector<std::string> checkpoints_;
};

template <typename RNG>
std::vector<Task> RandomTaskGraphMaker::build(
    RNG &&rng,
    InOut<std::size_t> run_count) {
  checkpoint_to_upstream_tasks_map_.clear();
  executed_tasks_.clear();

  TaskId current_task_id = 0;
  auto new_task_id = [&current_task_id]() mutable { return current_task_id++; };

  std::vector<Task> tasks;
  for (std::size_t ii = 0; ii < checkpoints_.size(); ++ii) {
    for (std::size_t jj = 0; jj < ii; ++jj) {
      if (random_bool(rng)) {
        continue;
      }
      const TaskId task_id{new_task_id()};
      tasks.push_back(Task{
          .name = "some_task",
          .dependency = checkpoints_.at(jj),
          .provision = checkpoints_.at(ii),
          .work =
              [task_id, this, dep = checkpoints_.at(jj), run_count]() mutable {
                // Check that every task providing our dependency has already
                // run.
                const auto [dep_start, dep_end] =
                    checkpoint_to_upstream_tasks_map_.equal_range(dep);
                for (auto it = dep_start; it != dep_end; ++it) {
                  EXPECT_TRUE(executed_tasks_.contains(it->second));
                }
                // Report that we've run
                executed_tasks_.insert(task_id);
                ++*run_count;
                return true;
              },
      });
      checkpoint_to_upstream_tasks_map_.emplace(checkpoints_.at(ii), task_id);
    }

    // If we don't have any upstream tasks, we need to add one or this
    // checkpoint will never be active.
    if (not checkpoint_to_upstream_tasks_map_.contains(checkpoints_.at(ii))) {
      const TaskId task_id{new_task_id()};
      tasks.push_back(Task{
          .name = "some_task",
          .dependency = std::nullopt,
          .provision = checkpoints_.at(ii),
          .work =
              [task_id, this, run_count]() mutable {
                // Report that we've run
                executed_tasks_.insert(task_id);
                ++*run_count;
                return true;
              },
      });
      checkpoint_to_upstream_tasks_map_.emplace(checkpoints_.at(ii), task_id);
    }
  }
  return tasks;
}

}  // namespace

TEST(SimpleStepExecutorTest, TestSimpleStepExecutorRandomGraph) {
  // SETUP
  constexpr int SEED = 29546U;
  std::mt19937 rng{SEED};

  RandomTaskGraphMaker graph_maker;

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    std::size_t run_count = 0U;
    auto tasks = graph_maker.build(rng, InOut{run_count});
    const std::size_t expected_run_count = tasks.size();

    // ACTION / VERIFICATION
    SimpleStepExecutor executor{std::move(tasks)};
    executor.run_step();

    // Check that every task ran:
    EXPECT_EQ(run_count, expected_run_count);
  }
}

TEST(SimpleStepExecutorTest, TestBranching) {
  // SETUP
  int run_count = 0;
  constexpr int EXPECTED_RUN_COUNT = 2;

  std::vector<Task> tasks = {
      Task{
          .name = "A",
          .dependency = std::nullopt,
          .provision = "checkpoint",
          .work =
              [&]() {
                ++run_count;
                return true;
              },
      },
      Task{
          .name = "B",
          .dependency = std::nullopt,
          .provision = "checkpoint",
          .work =
              [&]() {
                ++run_count;
                return false;
              },
      },
      Task{
          .name = "C",
          .dependency = "checkpoint",
          .provision = "null",
          .work =
              [&]() {
                ++run_count;
                // This should never run:
                EXPECT_FALSE(true);
                return true;
              },
      },
  };

  // ACTION / VERIFICATION
  SimpleStepExecutor executor{std::move(tasks)};
  executor.run_step();

  // Check that every task ran:
  EXPECT_EQ(run_count, EXPECTED_RUN_COUNT);
}

// Test that we don't run tasks whose dependency has no tasks providing it
TEST(SimpleStepExecutorTest, TestNoRunDisconnected) {
  // SETUP
  std::vector<Task> tasks = {Task{
      .name = "A",
      .dependency = "does_not_exist",
      .provision = "checkpoint",
      .work =
          [&]() {
            EXPECT_FALSE(true);  // Should never run
            return true;
          },
  }};

  // ACTION / VERIFICATION
  SimpleStepExecutor executor{std::move(tasks)};
  executor.run_step();
}

// Test that running the exector N times runs the tasks it contains N times.
TEST(SimpleStepExecutorTest, TestRunNTimes) {
  // SETUP
  int run_count = 0;
  std::vector<Task> tasks = {Task{
      .name = "A",
      .dependency = std::nullopt,
      .provision = "checkpoint",
      .work =
          [&]() {
            ++run_count;
            return true;
          },
  }};
  SimpleStepExecutor executor{std::move(tasks)};

  constexpr int NUM_RUNS = 10;
  for (int ii = 0; ii < NUM_RUNS; ++ii) {
    // ACTION
    executor.run_step();

    // VERIFICATION
    EXPECT_EQ(run_count, ii + 1);
  }
}

}  // namespace resim::simulator
