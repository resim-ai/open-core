
#pragma once

#include <Eigen/Dense>

#include "resim/math/vector_partition.hh"

namespace resim::planning::drone {

struct Control {
  enum BlockIndices : std::size_t {
    ANGULAR_ACCELERATION = 0,
    THRUST,
    NUM_BLOCKS,
  };
  using Partition = resim::math::VectorPartition<3, 1>;

  static constexpr int DIM = math::OffsetAt<Partition, NUM_BLOCKS>::value;
  using Vec = Eigen::Matrix<double, DIM, 1>;
  Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};
  double thrust = 0;
};

Control operator+(const Control &u, const typename Control::Vec &du);

typename Control::Vec operator-(const Control &u, const Control &v);

}  // namespace resim::planning::drone
