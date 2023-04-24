#pragma once

#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/state/rigid_body_state.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/framed_group.hh"

namespace resim::actor::state {

// This struct represents the observable state of an actor along with that
// actor's id and the state's time of validity.
struct ObservableState {
  ActorId id;
  bool is_spawned = false;
  time::Timestamp time_of_validity;
  RigidBodyState<transforms::FSE3> state;
};

}  // namespace resim::actor::state
