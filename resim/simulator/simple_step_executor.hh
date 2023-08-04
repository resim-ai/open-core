// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <functional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "resim/simulator/step_executor.hh"

namespace resim::simulator {

// A simple implementation of a StepExecutor which uses a single
// thread and pushes all tasks onto a work queue once they are ready
// to run.
class SimpleStepExecutor final : public StepExecutor {
 public:
  // Construct this step executor from a vector of tasks.
  explicit SimpleStepExecutor(std::vector<Task> &&tasks);

  // Run a step
  void run_step() override;

 private:
  // This helper data structure tracks how many predecessors (tasks that list it
  // as a provision) of a given dependency have completed and what tasks
  // (successors) depend on this dependency.
  class CheckPoint {
   public:
    using TaskIdx = std::size_t;
    CheckPoint() = default;

    // Resets this checkpoint meaning that every predecessor must now
    // complete before this checkpoint's successors are ready to
    // execute again.
    void reset();

    // Add a successor to this checkpoint.
    void add_successor(TaskIdx successor);

    // Add a predecessor to this checkpoint. We don't need to track
    // its index in this case.
    void add_predecessor();

    // Takes the most recently finished predecessor index and
    // determines whether this checkpoint's successors are now ready
    // to execute.
    // @param[in] finished_pred - The predecessor which has just finished.
    bool is_ready(TaskIdx finished_pred);

    // Getter for the successors.
    const std::vector<TaskIdx> &successors() const;

   private:
    std::vector<TaskIdx> successors_;

    // The number of unique predecessors which have completed
    TaskIdx predecessor_count_{0U};

    // The set of predecessors which have completed
    std::unordered_set<TaskIdx> complete_predecessors_;
  };

  // Add a checkpoint's successors to the work queue.
  void queue_checkpoint(const CheckPoint &checkpoint);

  // A special checkpoint representing the start. All tasks which do
  // not depend on any checkpoints depend on this one as an implementation
  // detail.
  CheckPoint start_;

  std::unordered_map<std::string_view, CheckPoint> checkpoints_;
  std::vector<Task> tasks_;

  // A queue to handle the actual running of all of our tasks.
  std::queue<std::function<void()>> work_queue_;
};

}  // namespace resim::simulator
