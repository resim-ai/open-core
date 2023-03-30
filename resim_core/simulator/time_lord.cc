
#include "resim_core/simulator/time_lord.hh"

#include "resim_core/assert/assert.hh"

namespace resim::simulator {

TimeLord::TimeLord(InOut<ExecutorBuilder> executor_builder) {
  executor_builder->add_independent_task<time::Timestamp>(
      "publish_time",
      TIME_TOPIC,
      [this]() { return current_time_; });
}

void TimeLord::increment_time(const time::Duration dt) {
  constexpr time::Duration ZERO_TIME{};
  REASSERT(dt > ZERO_TIME, "Times must strictly advance!");
  current_time_ += dt;
}

}  // namespace resim::simulator
