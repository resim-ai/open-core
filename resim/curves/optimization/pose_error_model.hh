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

struct TimedPose {
  double time = 0.0;
  transforms::SE3 observation_from_scene;
};

class PoseErrorModel : public math::ErrorModel {
 public:
  PoseErrorModel(
      const math::GaussNewtonOptimizer &optimizer,
      std::string key,
      std::vector<TimedPose> poses);

  int dof() const override;

  void operator()(
      Eigen::VectorBlock<Eigen::VectorXd> error_block,
      const JacobianWriter &error_jacobian_writer) const override;

 private:
  const math::GaussNewtonOptimizer &optimizer_;
  std::string key_;
  std::vector<TimedPose> poses_;
};

}  // namespace resim::curves::optimization
