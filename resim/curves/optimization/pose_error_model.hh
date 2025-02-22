// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>
#include <string>
#include <utility>

#include "resim/math/gauss_newton_optimizer.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves::optimization {

// A simple structure representing a pose observation at a given time.
struct TimedPose {
  double time = 0.0;
  transforms::SE3 observation_from_scene;
};

// An error model which expects a
// resim::math::ParameterBlock<ControlPointParameter> to exist in the
// GaussNewtonOptimizer at the given key. This block is interpreted as a TCurve
// and it is optimized to match the observed sequence of poses.
class PoseErrorModel : public math::ErrorModel {
 public:
  // Constructor
  // @param[in] optimizer - The optimizer with which we intend to register this
  //                        model.
  // @param[in] key - The key of the ParameterBlock<ControlPointParameter> to
  //                  treat as a TCurve.
  // @param[in] poses - The observation poses to optimize against.
  PoseErrorModel(
      const math::GaussNewtonOptimizer &optimizer,
      std::string key,
      std::vector<TimedPose> poses);

  // The number of error degrees of freedom.
  int dof() const override;

  // The evaluation operation for this error model.
  void operator()(
      Eigen::VectorBlock<Eigen::VectorXd> error_block,
      const JacobianWriter &error_jacobian_writer) const override;

 private:
  const math::GaussNewtonOptimizer &optimizer_;
  std::string key_;
  std::vector<TimedPose> poses_;
};

}  // namespace resim::curves::optimization
