
#pragma once

#include <memory>

#include "resim/simulator/executor_builder.hh"
#include "resim/simulator/simulation_unit.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/mcap_logger.hh"

namespace resim::simulator {

// A simple SimulationUnit used to manage the time in a simulation.
class TimeLord final : public SimulationUnit {
 public:
  // Add this SimulationUnit's tasks to the given executor_builder.
  // @param[inout] executor_builder - The ExecutorBuilder to add the
  //                                  tasks to.
  explicit TimeLord(
      std::shared_ptr<LoggerInterface> logger_interface,
      InOut<ExecutorBuilder> executor_builder);

  // Advance the current time by the given amount.
  // @param[in] The time to advance. Must be positive.
  void increment_time(time::Duration dt);

 private:
  time::Timestamp current_time_;
};

}  // namespace resim::simulator
