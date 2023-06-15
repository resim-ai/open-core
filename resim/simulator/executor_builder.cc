
#include "resim/simulator/executor_builder.hh"

#include "resim/simulator/simple_step_executor.hh"

namespace resim::simulator {

std::unique_ptr<StepExecutor> ExecutorBuilder::build() {
  auto result = std::make_unique<SimpleStepExecutor>(std::move(tasks_));
  tasks_ = std::vector<Task>{};
  channel_registry_.reset();
  return result;
}

ExecutorBuilder &ExecutorBuilder::add_independent_task(
    const std::string_view &name,
    const std::string_view &provision,
    std::function<void()> &&task) {
  tasks_.push_back(Task{
      .name = name,
      .dependency = std::nullopt,
      .provision = provision,
      .work =
          [task = std::move(task)]() {
            task();
            return true;
          },
  });
  return *this;
}

ExecutorBuilder &ExecutorBuilder::add_task(
    const std::string_view &name,
    const std::string_view &dependency,
    const std::string_view &provision,
    std::function<void()> &&task) {
  tasks_.push_back(Task{
      .name = name,
      .dependency = dependency,
      .provision = provision,
      .work =
          [task = std::move(task)]() {
            task();
            return true;
          },
  });
  return *this;
}

ExecutorBuilder &ExecutorBuilder::add_conditional_task(
    const std::string_view &name,
    const std::string_view &dependency,
    const std::string_view &provision,
    std::function<bool()> &&task) {
  tasks_.push_back(Task{
      .name = name,
      .dependency = dependency,
      .provision = provision,
      .work = [task = std::move(task)]() { return task(); },
  });
  return *this;
}

}  // namespace resim::simulator
