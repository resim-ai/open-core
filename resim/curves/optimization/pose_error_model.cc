// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/optimization/pose_error_model.hh"
#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves::optimization {

int PoseErrorModel::dof() const {
  return num_points_ * TWO_JET_DOF<transforms::SE3>;
}

void PoseErrorModel::operator()(
    Eigen::VectorBlock<Eigen::VectorXd> error_block,
    const JacobianWriter &error_jacobian_writer) const {
  // TODO
}

}  // namespace resim::curves::optimization
