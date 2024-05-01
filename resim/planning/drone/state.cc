

#include "resim/planning/drone/state.hh"

namespace resim::planning::drone {

using math::get_block;

State operator+(const State &x, const typename State::Vec &dx) {
  return State{
      .scene_from_body_rotation =
          x.scene_from_body_rotation *
          transforms::SO3::exp(get_block<StatePartition, ROTATION>(dx)),
      .position = x.position + get_block<StatePartition, POSITION>(dx),
      .angular_velocity =
          x.angular_velocity + get_block<StatePartition, ANGULAR_VELOCITY>(dx),
      .velocity = x.velocity + get_block<StatePartition, VELOCITY>(dx),
  };
}

typename State::Vec operator-(const State &x, const State &y) {
  State::Vec result;
  get_block<StatePartition, ROTATION>(result) =
      (y.scene_from_body_rotation.inverse() * x.scene_from_body_rotation).log();
  get_block<StatePartition, POSITION>(result) = x.position - y.position;
  get_block<StatePartition, ANGULAR_VELOCITY>(result) =
      x.angular_velocity - y.angular_velocity;
  get_block<StatePartition, VELOCITY>(result) = x.velocity - y.velocity;
  return result;
}

}  // namespace resim::planning::drone
