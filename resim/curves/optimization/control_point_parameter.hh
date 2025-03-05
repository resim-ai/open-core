// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>

#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/curves/t_curve.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves::optimization {

// A simple struct containing a TCurve<SE3>::Control and giving it the DOF
// parameter and accumulate() function required by
// resim::math::GaussNewtonOptimizer.
struct ControlPointParameter {
  static constexpr int DOF = TWO_JET_DOF<transforms::SE3>;

  ControlPointParameter accumulate(
      const Eigen::Ref<const Eigen::VectorXd>& delta) const;

  TCurve<transforms::SE3>::Control value;
};
}  // namespace resim::curves::optimization
