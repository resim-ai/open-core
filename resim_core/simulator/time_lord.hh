
#pragma once

#include <string_view>

#include "resim_core/simulator/executor_builder.hh"
#include "resim_core/simulator/simulation_unit.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/utils/inout.hh"

namespace resim::simulator {

// A simple SimulationUnit used to manage the time in a simulation.
class TimeLord final : public SimulationUnit {
 public:
  static constexpr std::string_view TIME_TOPIC = "time";

  // Add this SimulationUnit's tasks to the given executor_builder.
  // @param[inout] executor_builder - The ExecutorBuilder to add the
  //                                  tasks to.
  explicit TimeLord(InOut<ExecutorBuilder> executor_builder);

  // Advance the current time by the given amount.
  // @param[in] The time to advance. Must be positive.
  void increment_time(time::Duration dt);

 private:
  time::Timestamp current_time_;
};

}  // namespace resim::simulator
