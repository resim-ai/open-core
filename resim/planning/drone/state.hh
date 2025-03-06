// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>

#include "resim/math/vector_partition.hh"
#include "resim/transforms/so3.hh"

namespace resim::planning::drone {

struct State {
  enum BlockIndices : std::size_t {
    ROTATION = 0,
    POSITION,
    ANGULAR_VELOCITY,
    VELOCITY,
    TIME,
    NUM_BLOCKS,
  };

  using Partition = resim::math::VectorPartition<3, 3, 3, 3, 1>;

  static constexpr int DIM = math::OffsetAt<Partition, NUM_BLOCKS>::value;
  using Vec = Eigen::Matrix<double, DIM, 1>;

  // We don't use SE3 directly for this because the cost functions get
  // un-necessarily complicated to formulate in SE3.
  transforms::SO3 scene_from_body_rotation;
  Eigen::Vector3d position;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d velocity;
  Eigen::Matrix<double, 1, 1> time;
};

State operator+(const State &x, const typename State::Vec &dx);

typename State::Vec operator-(const State &x, const State &y);

}  // namespace resim::planning::drone
