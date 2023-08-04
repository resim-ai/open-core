// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "resim/assert/assert.hh"
#include "resim/simulator/channel_registry.hh"
#include "resim/simulator/step_executor.hh"

namespace resim::simulator {

// This class's job is to create a vector of tasks that can be handled by a
// `StepExecutor` and it acts as a factory for an abstract `StepExecutor` via
// the `ExecutorBuilder::build()` function. To this end, it implements a fluent
// api that allows users to add tasks using many different methods depending on
// whether the tasks have inputs or outputs.
class ExecutorBuilder final {
 public:
  using Task = StepExecutor::Task;

  // Build the step executor using the currently added tasks and reset our
  // tasks.
  std::unique_ptr<StepExecutor> build();

  // Add a task which is independent meaning it runs on every step and doesn't
  // accept any inputs. In this overload the task provides no output.
  // @param[in] name - A name for this task.
  // @param[in] provision - A tag that other tasks which must run after this
  //                        task can list as their dependency.
  // @param[in] task - The task to run.
  ExecutorBuilder &add_independent_task(
      const std::string_view &name,
      const std::string_view &provision,
      std::function<void()> &&task);

  // Add a task which is independent meaning it runs on every step and doesn't
  // accept any inputs. In this overload the task returns a value which
  // dependents of its provision can accept as input.
  // @param[in] name - A name for this task.
  // @param[in] provision - A tag that other tasks which must run after this
  //                        task can list as their dependency.
  // @param[in] task - The task to run.
  template <typename Ret>
  ExecutorBuilder &add_independent_task(
      const std::string_view &name,
      const std::string_view &provision,
      std::function<Ret()> &&task);

  // Add a task which runs when all of the tasks it depends on complete
  // successfully. In this overload, the task provides no output, and is passed
  // a vector of the results of all of the tasks it depends on directly which
  // choose to output values.
  // provision can accept as input.
  // @param[in] name - A name for this task.
  // @param[in] dependency - A tag that other tasks which must run before this
  //                         task can list as their provision.
  // @param[in] provision - A tag that other tasks which must run after this
  //                        task can list as their dependency.
  // @param[in] task - The task to run.
  template <typename Arg>
  ExecutorBuilder &add_task(
      const std::string_view &name,
      const std::string_view &dependency,
      const std::string_view &provision,
      std::function<void(const std::vector<Arg> &arg)> &&task);

  // Add a task which runs when all of the tasks it depends on complete
  // successfully. In this overload, the task provides an output, and is passed
  // a vector of the results of all of the tasks it depends on directly which
  // choose to output values.
  // provision can accept as input.
  // @param[in] name - A name for this task.
  // @param[in] dependency - A tag that other tasks which must run before this
  //                         task can list as their provision.
  // @param[in] provision - A tag that other tasks which must run after this
  //                        task can list as their dependency.
  // @param[in] task - The task to run.
  template <typename Arg, typename Ret>
  ExecutorBuilder &add_task(
      const std::string_view &name,
      const std::string_view &dependency,
      const std::string_view &provision,
      std::function<Ret(const std::vector<Arg> &arg)> &&task);

  // Overload of the above no-output case in which the task acepts only a single
  // value. This task will throw if there are multiple providers providing data
  // for its dependency (i.e. if the vector passed to the above overload would
  // have size greater than one).
  // @param[in] name - A name for this task.
  // @param[in] dependency - A tag that other tasks which must run before this
  //                         task can list as their provision.
  // @param[in] provision - A tag that other tasks which must run after this
  //                        task can list as their dependency.
  // @param[in] task - The task to run.
  template <typename Arg>
  ExecutorBuilder &add_task(
      const std::string_view &name,
      const std::string_view &dependency,
      const std::string_view &provision,
      std::function<void(const Arg &arg)> &&task);

  // Overload of the above output case in which the task acepts only a single
  // value. This task will throw if there are multiple providers providing data
  // for its dependency (i.e. if the vector passed to the above overload would
  // have size greater than one).
  // @param[in] name - A name for this task.
  // @param[in] dependency - A tag that other tasks which must run before this
  //                         task can list as their provision.
  // @param[in] provision - A tag that other tasks which must run after this
  //                        task can list as their dependency.
  // @param[in] task - The task to run.
  template <typename Arg, typename Ret>
  ExecutorBuilder &add_task(
      const std::string_view &name,
      const std::string_view &dependency,
      const std::string_view &provision,
      std::function<Ret(const Arg &arg)> &&task);

  // This overload works for tasks that have no data input or output but still
  // must run after some tasks (dependency), and possibly before others
  // (provision).
  // @param[in] name - A name for this task.
  // @param[in] dependency - A tag that other tasks which must run before this
  //                         task can list as their provision.
  // @param[in] provision - A tag that other tasks which must run after this
  //                        task can list as their dependency.
  // @param[in] task - The task to run.
  ExecutorBuilder &add_task(
      const std::string_view &name,
      const std::string_view &dependency,
      const std::string_view &provision,
      std::function<void()> &&task);

  // This overload works for tasks that have no data input but still
  // must run after some tasks (dependency) and before others.
  // @param[in] name - A name for this task.
  // @param[in] dependency - A tag that other tasks which must run before this
  //                         task can list as their provision.
  // @param[in] provision - A tag that other tasks which must run after this
  //                        task can list as their dependency.
  // @param[in] task - The task to run.
  template <typename Ret>
  ExecutorBuilder &add_task(
      const std::string_view &name,
      const std::string_view &dependency,
      const std::string_view &provision,
      std::function<Ret()> &&task);

  // Add a conditional task which only allows its dependencies to run if the
  // given task function returns true. This allows for branching logic.
  // @param[in] name - A name for this task.
  // @param[in] dependency - A tag that other tasks which must run before this
  //                         task can list as their provision.
  // @param[in] provision - A tag that other tasks which must run after this
  //                        task can list as their dependency.
  // @param[in] task - The task to run.
  ExecutorBuilder &add_conditional_task(
      const std::string_view &name,
      const std::string_view &dependency,
      const std::string_view &provision,
      std::function<bool()> &&task);

 private:
  ChannelRegistry channel_registry_;
  std::vector<Task> tasks_;
};

template <typename Ret>
ExecutorBuilder &ExecutorBuilder::add_independent_task(
    const std::string_view &name,
    const std::string_view &provision,
    std::function<Ret()> &&task) {
  auto publisher = channel_registry_.make_publisher<Ret>(provision);
  tasks_.push_back(Task{
      .name = name,
      .dependency = std::nullopt,
      .provision = provision,
      .work =
          [publisher = std::move(publisher), task = std::move(task)]() {
            publisher->publish(task());
            return true;
          },
  });
  return *this;
}

template <typename Arg, typename Ret>
ExecutorBuilder &ExecutorBuilder::add_task(
    const std::string_view &name,
    const std::string_view &dependency,
    const std::string_view &provision,
    std::function<Ret(const std::vector<Arg> &arg)> &&task) {
  auto channel = channel_registry_.channel<Arg>(dependency);
  auto publisher = channel_registry_.make_publisher<Ret>(provision);
  tasks_.push_back(Task{
      .name = name,
      .dependency = dependency,
      .provision = provision,
      .work =
          [input = std::move(channel),
           publisher = std::move(publisher),
           task = std::move(task)]() {
            publisher->publish(task(input->data()));
            return true;
          },
  });
  return *this;
}

template <typename Arg, typename Ret>
ExecutorBuilder &ExecutorBuilder::add_task(
    const std::string_view &name,
    const std::string_view &dependency,
    const std::string_view &provision,
    std::function<Ret(const Arg &arg)> &&task) {
  auto channel = channel_registry_.channel<Arg>(dependency);
  auto publisher = channel_registry_.make_publisher<Ret>(provision);
  tasks_.push_back(Task{
      .name = name,
      .dependency = dependency,
      .provision = provision,
      .work =
          [input = std::move(channel),
           publisher = std::move(publisher),
           task = std::move(task)]() {
            constexpr auto ERROR_MSG =
                "Topic doesn't have exactly one publisher!";
            REASSERT(input->data().size() == 1, ERROR_MSG);
            publisher->publish(task(input->data().front()));
            return true;
          },
  });
  return *this;
}

template <typename Arg>
ExecutorBuilder &ExecutorBuilder::add_task(
    const std::string_view &name,
    const std::string_view &dependency,
    const std::string_view &provision,
    std::function<void(const std::vector<Arg> &arg)> &&task) {
  auto channel = channel_registry_.channel<Arg>(dependency);
  tasks_.push_back(Task{
      .name = name,
      .dependency = dependency,
      .provision = provision,
      .work =
          [input = std::move(channel), task = std::move(task)]() {
            task(input->data());
            return true;
          },
  });
  return *this;
}

template <typename Arg>
ExecutorBuilder &ExecutorBuilder::add_task(
    const std::string_view &name,
    const std::string_view &dependency,
    const std::string_view &provision,
    std::function<void(const Arg &arg)> &&task) {
  auto channel = channel_registry_.channel<Arg>(dependency);
  tasks_.push_back(Task{
      .name = name,
      .dependency = dependency,
      .provision = provision,
      .work =
          [input = std::move(channel), task = std::move(task)]() {
            constexpr auto ERROR_MSG =
                "Topic doesn't have exactly one publisher!";
            REASSERT(input->data().size() == 1, ERROR_MSG);
            task(input->data().front());
            return true;
          },
  });
  return *this;
}

template <typename Ret>
ExecutorBuilder &ExecutorBuilder::add_task(
    const std::string_view &name,
    const std::string_view &dependency,
    const std::string_view &provision,
    std::function<Ret()> &&task) {
  auto publisher = channel_registry_.make_publisher<Ret>(provision);
  tasks_.push_back(Task{
      .name = name,
      .dependency = dependency,
      .provision = provision,
      .work =
          [publisher = std::move(publisher), task = std::move(task)]() {
            publisher->publish(task());
            return true;
          },
  });
  return *this;
}

}  // namespace resim::simulator
