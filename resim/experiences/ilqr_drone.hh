// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>

namespace resim::experiences {

// Empty for now: to be populated with the iLQR drone model.
struct ILQRDrone {
  double velocity_cost = 0.0;
  Eigen::Vector3d initial_position;
  Eigen::Vector3d goal_position;
};

}  // namespace resim::experiences
