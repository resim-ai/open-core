
#pragma once

#include <Eigen/Dense>

#include "resim/math/vector_partition.hh"
#include "resim/transforms/so3.hh"

namespace resim::planning::drone {

enum StateBlockIndices : std::size_t {
  ROTATION = 0,
  POSITION,
  ANGULAR_VELOCITY,
  VELOCITY,
  NUM_BLOCKS,
};

using StatePartition = resim::math::VectorPartition<3, 3, 3, 3>;

struct State {
  static constexpr int DIM = math::OffsetAt<StatePartition, NUM_BLOCKS>::value;
  using Vec = Eigen::Matrix<double, DIM, 1>;

  // We don't use SE3 directly for this because the cost functions get
  // un-necessarily complicated to formulate in SE3.
  transforms::SO3 scene_from_body_rotation;
  Eigen::Vector3d position;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d velocity;
};

State operator+(const State &x, const typename State::Vec &dx);

typename State::Vec operator-(const State &x, const State &y);

}  // namespace resim::planning::drone
