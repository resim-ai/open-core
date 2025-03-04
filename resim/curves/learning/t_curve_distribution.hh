// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>

#include "resim/curves/t_curve.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves::learning {

struct TCurveDistribution {
  TCurve<transforms::SE3> mean;
  Eigen::MatrixXd covariance{Eigen::MatrixXd::Zero(0, 0)};
};

}  // namespace resim::curves::learning
