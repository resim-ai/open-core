#pragma once

#include <memory>

#include "resim_core/actor/actor.hh"
#include "resim_core/simulator/executor_builder.hh"
#include "resim_core/simulator/simulation_unit.hh"
#include "resim_core/utils/inout.hh"
#include "resim_core/utils/mcap_logger.hh"

namespace resim::actor {

const std::string LOG_ACTOR_STATES_TOPIC{"log_actor_states"};
const std::string LOG_ACTOR_GEOMETRIES_TOPIC{"log_actor_geometries"};

// This class provides a simulation unit wrapping an abstract actor. It
// coordinates the receipt of simulation timestamps by the actor and the
// publication of the resulting observable actor states.
class ActorUnit : public simulator::SimulationUnit {
 public:
  explicit ActorUnit(
      std::shared_ptr<LoggerInterface> logger_interface,
      std::unique_ptr<Actor> actor,
      InOut<simulator::ExecutorBuilder> executor_builder);

 private:
  std::unique_ptr<Actor> actor_;

  // A private dependency string used to enforce that simulate forward always
  // runs before actor state and geometry publication.
  const std::string simulate_forward_dependency_;
};

// This unit logs the output states and geometries published by all actor units
class ActorLoggerUnit : public simulator::SimulationUnit {
 public:
  explicit ActorLoggerUnit(
      std::shared_ptr<LoggerInterface> logger_interface,
      InOut<simulator::ExecutorBuilder> executor_builder);

 private:
  // Helper function to log the given set of actor states
  void log_actor_states(
      const std::vector<actor::state::ObservableState> &actor_states);

  // Helper function to log the given set of actor geometries
  void log_geometries_update(
      const std::vector<actor::Geometry> &actor_geometries) const;
};

}  // namespace resim::actor
