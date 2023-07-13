
#pragma once

#include <memory>
#include <queue>

#include "resim/simulator/executor_builder.hh"
#include "resim/simulator/simulation_unit.hh"
#include "resim/simulator/time_lord_update.hh"
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

  void schedule_update(const TimeLordUpdate& update);

  bool ready_to_terminate();

 private:
  std::priority_queue<
      TimeLordUpdate,
      std::vector<TimeLordUpdate>,
      std::greater<>>
      scheduled_updates_;
  std::optional<time::Timestamp> current_time_ = std::nullopt;
  bool terminate_flag_ = false;
  ;
};

}  // namespace resim::simulator
