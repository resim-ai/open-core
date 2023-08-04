// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <functional>
#include <optional>
#include <string_view>

namespace resim::simulator {

// This library represents the interface for a class responsible for running a
// set of given tasks while respecting order dependence on each step. In
// particular, each `Task` is specified with a name, an optional dependency
// string, a provision string, and a piece of work to execute. Here's how this
// works. A given task may run if and only if all tasks whose provisions match
// its dependency have previously run. If the dependency is `std::nullopt`, then
// the task will run at an arbitrary time within the step, although any tasks
// depending on its provision will run after it to comply with the previous
// requirement. If the task's dependency is not provided by any other task, **it
// will not run**. Furthermore, each step should run independently. For
// instance, completion of a task on a previous step should not affect its
// dependents on future steps. The work function returns a boolean reflecting
// whether or not it was successful. This allows us to support rudimentary
// branching logic (although the `false` branch just does nothing in this case),
// which is convenient for our use case. This interface is written so that it
// could work with a third-party parallel task scheduler like
// [Taskflow](https://taskflow.github.io/taskflow/index.html) or Intel's
// [Threading Building Blocks](https://github.com/oneapi-src/oneTBB).
class StepExecutor {
 public:
  // A struct representing a task as described above.
  struct Task {
    std::string_view name;
    std::optional<std::string_view> dependency;
    std::string_view provision;

    using Work = std::function<bool()>;
    Work work;
  };

  StepExecutor() = default;
  virtual ~StepExecutor() = default;
  StepExecutor(const StepExecutor &) = default;
  StepExecutor(StepExecutor &&) = default;
  StepExecutor &operator=(const StepExecutor &) = default;
  StepExecutor &operator=(StepExecutor &&) = default;

  // Run the tasks for this step as described above.
  virtual void run_step() = 0;
};

}  // namespace resim::simulator
