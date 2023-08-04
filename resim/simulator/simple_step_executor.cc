// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#include "resim/simulator/simple_step_executor.hh"

namespace resim::simulator {

bool SimpleStepExecutor::CheckPoint::is_ready(const TaskIdx finished_pred) {
  complete_predecessors_.insert(finished_pred);
  // Reset and return true if we've received from all predecessors.
  if (complete_predecessors_.size() == predecessor_count_) {
    reset();
    return true;
  }
  return false;
}

void SimpleStepExecutor::CheckPoint::add_successor(const TaskIdx successor) {
  successors_.push_back(successor);
}

void SimpleStepExecutor::CheckPoint::reset() { complete_predecessors_.clear(); }

const std::vector<SimpleStepExecutor::CheckPoint::TaskIdx>&
SimpleStepExecutor::CheckPoint::successors() const {
  return successors_;
}

void SimpleStepExecutor::CheckPoint::add_predecessor() { ++predecessor_count_; }

SimpleStepExecutor::SimpleStepExecutor(std::vector<Task>&& tasks)
    : tasks_{std::move(tasks)} {
  for (std::size_t ii = 0U; ii < tasks_.size(); ++ii) {
    const Task& task = tasks_.at(ii);
    if (task.dependency.has_value()) {
      checkpoints_[task.dependency.value()].add_successor(ii);
    } else {
      start_.add_successor(ii);
    }

    checkpoints_[task.provision].add_predecessor();
  }
}

void SimpleStepExecutor::queue_checkpoint(const CheckPoint& checkpoint) {
  for (const auto successor : checkpoint.successors()) {
    work_queue_.push([this, successor]() {
      const auto& task = tasks_.at(successor);
      // If the task's work function returns true AND this provision checkpoint
      // is now ready, then we can queue its provision checkpoint. Otherwise,
      // keep running.
      if (task.work() && checkpoints_.at(task.provision).is_ready(successor)) {
        queue_checkpoint(checkpoints_.at(task.provision));
      }
    });
  }
}

void SimpleStepExecutor::run_step() {
  // Reset all checkpoints so we can't rely on values from the previous step.
  for (auto& [_, checkpoint] : checkpoints_) {
    checkpoint.reset();
  }
  queue_checkpoint(start_);

  // Keep spinning until we've completed all work.
  while (not work_queue_.empty()) {
    work_queue_.front()();
    work_queue_.pop();
  }
}

}  // namespace resim::simulator
