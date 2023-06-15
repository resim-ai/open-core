
#include "resim/simulator/time_lord.hh"

#include "resim/assert/assert.hh"
#include "resim/simulator/simulation_unit.hh"
#include "resim/simulator/standard_topics.hh"

namespace resim::simulator {

TimeLord::TimeLord(
    std::shared_ptr<LoggerInterface> logger_interface,
    InOut<ExecutorBuilder> executor_builder)
    : SimulationUnit(std::move(logger_interface)) {
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
