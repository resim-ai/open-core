// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/planning/drone/state.hh"

namespace resim::planning::drone {

using math::get_block;

State operator+(const State &x, const typename State::Vec &dx) {
  return State{
      .scene_from_body_rotation =
          x.scene_from_body_rotation *
          transforms::SO3::exp(
              get_block<State::Partition, State::ROTATION>(dx)),
      .position = x.position + get_block<State::Partition, State::POSITION>(dx),
      .angular_velocity =
          x.angular_velocity +
          get_block<State::Partition, State::ANGULAR_VELOCITY>(dx),
      .velocity = x.velocity + get_block<State::Partition, State::VELOCITY>(dx),
  };
}

typename State::Vec operator-(const State &x, const State &y) {
  State::Vec result;
  get_block<State::Partition, State::ROTATION>(result) =
      (y.scene_from_body_rotation.inverse() * x.scene_from_body_rotation).log();
  get_block<State::Partition, State::POSITION>(result) =
      x.position - y.position;
  get_block<State::Partition, State::ANGULAR_VELOCITY>(result) =
      x.angular_velocity - y.angular_velocity;
  get_block<State::Partition, State::VELOCITY>(result) =
      x.velocity - y.velocity;
  return result;
}

}  // namespace resim::planning::drone
