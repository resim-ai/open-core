#pragma once

#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/geometry.hh"
#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/time/timestamp.hh"

namespace resim::actor {

// This interface wraps any kind of actor which can be simulated forward and
// which has an observable rigid body state.
class Actor {
 public:
  explicit Actor(ActorId id);
  Actor(const Actor &) = default;
  Actor(Actor &&) = default;
  Actor &operator=(const Actor &) = default;
  Actor &operator=(Actor &&) = default;
  virtual ~Actor() = default;

  // Getter for the actor's id
  ActorId id() const;

  // Simulate this actor forward to the given time. This function should only be
  // called with non-decreasing times.
  //
  // Note: We may eventually have some actors support event localization by
  // expanding this interface to have a similar function for attempting steps.
  //
  // @param[in] time - The time to advance to
  virtual void simulate_forward(time::Timestamp time) = 0;

  // Get the current observable state of this actor. Not returned by reference
  // because actors may not store their observable state, opting rather to
  // compute it when requested.
  virtual state::ObservableState observable_state() const = 0;

  // Get the current geometry of the actor. This should normally contain
  // Geometry::NoUpdate for the actor's frame, but should update the geometry
  // when the actor spawns and Geometry::Clear it when the actor despawns.
  virtual Geometry geometry() const = 0;

  // Get the current time of this actor (i.e. the last time with which
  // simulate_forward was called.
  virtual time::Timestamp current_time() const = 0;

 private:
  ActorId id_;
};

}  // namespace resim::actor
