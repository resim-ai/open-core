#pragma once

#include <memory>

#include "resim_core/actor/actor.hh"
#include "resim_core/simulator/executor_builder.hh"
#include "resim_core/simulator/simulation_unit.hh"
#include "resim_core/utils/inout.hh"

namespace resim::actor {

// This class provides a simulation unit wrapping an abstract actor. It
// coordinates the receipt of simulation timestamps by the actor and the
// publication of the resulting observable actor states.
class ActorUnit : public simulator::SimulationUnit {
 public:
  explicit ActorUnit(
      std::unique_ptr<Actor> actor,
      InOut<simulator::ExecutorBuilder> executor_builder);

 private:
  std::unique_ptr<Actor> actor_;

  // A private dependency string used to enforce that simulate forward always
  // runs before actor state and geometry publication.
  const std::string simulate_forward_dependency_;
};

}  // namespace resim::actor
