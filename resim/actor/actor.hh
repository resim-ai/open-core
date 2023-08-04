// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/actor/actor_id.hh"
#include "resim/actor/state/observable_state.hh"
#include "resim/time/timestamp.hh"

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

  // Get the current time of this actor (i.e. the last time with which
  // simulate_forward was called.
  virtual time::Timestamp current_time() const = 0;

 private:
  ActorId id_;
};

}  // namespace resim::actor
