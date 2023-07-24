#pragma once

#include <memory>

#include "resim/experiences/completion_criteria.hh"
#include "resim/experiences/location_condition.hh"
#include "resim/simulator/executor_builder.hh"
#include "resim/simulator/simulation_unit.hh"
#include "resim/utils/inout.hh"

namespace resim::actor {

// This class adds a unit which checks for termination conditions at
// each timestamp, and schedules termination when one is hit - with an optional
// delay. (Note that completion criteria currently use OR semantics.)
class LocationConditionUnit : public simulator::SimulationUnit {
 public:
  explicit LocationConditionUnit(
      std::shared_ptr<LoggerInterface> logger_interface,
      const experiences::CompletionCriteria &criteria,
      InOut<simulator::ExecutorBuilder> executor_builder);
};
}  // namespace resim::actor
