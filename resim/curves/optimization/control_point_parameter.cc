// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/optimization/control_point_parameter.hh"

namespace resim::curves::optimization {

ControlPointParameter ControlPointParameter::accumulate(
    const Eigen::Ref<const Eigen::VectorXd>& delta) const {
  return ControlPointParameter{
      .value =
          TCurve<transforms::SE3>::Control{
              .time = value.time,
              .point = optimization::accumulate(value.point, delta),
          },
  };
}

}  // namespace resim::curves::optimization
