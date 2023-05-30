
#pragma once

#include <memory>
#include <vector>

#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/simulator/executor_builder.hh"
#include "resim_core/simulator/simulation_unit.hh"
#include "resim_core/utils/inout.hh"

namespace resim::metrics {

// This is a proof-of-concept logger unit for metrics to be computed per actor,
// during sim run time.
class ActorMetricsUnit : public simulator::SimulationUnit {
 public:
  ActorMetricsUnit(
      std::shared_ptr<LoggerInterface> logger_interface,
      InOut<simulator::ExecutorBuilder> executor_builder,
      actor::ActorId actor_id);

 private:
  void log_actor_metrics_(
      const std::vector<actor::state::ObservableState> &actor_states);

  void log_actor_min_distance_metric_(
      const std::vector<actor::state::ObservableState> &actor_states);

  const actor::ActorId actor_id_;
};

}  // namespace resim::metrics
