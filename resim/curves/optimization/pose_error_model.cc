// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/optimization/pose_error_model.hh"

#include <algorithm>
#include <iostream>
#include <iterator>

#include "resim/assert/assert.hh"
#include "resim/curves/optimization/control_point_parameter.hh"
#include "resim/curves/optimization/t_curve_differential.hh"
#include "resim/curves/optimization/two_jet_tangent_space.hh"

namespace resim::curves::optimization {

using transforms::SE3;

PoseErrorModel::PoseErrorModel(
    const math::GaussNewtonOptimizer &optimizer,
    std::string key,
    std::vector<TimedPose> poses)
    : optimizer_{optimizer},
      key_{std::move(key)},
      poses_{std::move(poses)} {}

int PoseErrorModel::dof() const {
  return static_cast<int>(poses_.size() * transforms::SE3::DOF);
}

void PoseErrorModel::operator()(
    Eigen::VectorBlock<Eigen::VectorXd> error_block,
    const JacobianWriter &error_jacobian_writer) const {
  const auto t_curve_control_block =
      optimizer_.get_parameters<ControlPointParameter>(key_);

  REASSERT(
      not t_curve_control_block.data.empty(),
      "Encountered invalid parameter block");

  REASSERT(std::is_sorted(
      t_curve_control_block.data.cbegin(),
      t_curve_control_block.data.cend(),
      [](const ControlPointParameter &a, const ControlPointParameter &b) {
        return a.value.time < b.value.time;
      }));

  for (std::size_t ii = 0U; ii < poses_.size(); ++ii) {
    const auto &pose = poses_.at(ii);

    REASSERT(
        t_curve_control_block.data.front().value.time <= pose.time,
        "Pose time outside of parameter block range");
    REASSERT(
        pose.time <= t_curve_control_block.data.back().value.time,
        "Pose time outside of parameter block range");

    auto next_it = std::upper_bound(
        t_curve_control_block.data.cbegin(),
        t_curve_control_block.data.cend(),
        pose.time,
        [](const double time, const ControlPointParameter &p) {
          return time < p.value.time;
        });

    if (next_it == t_curve_control_block.data.cend()) {
      --next_it;
    }
    const auto prev_it = std::prev(next_it);

    const auto point = point_at<SE3>(pose.time, prev_it->value, next_it->value);

    const auto idx = [&t_curve_control_block](const auto it) {
      return static_cast<int>(
          std::distance(t_curve_control_block.data.cbegin(), it));
    };

    const int offset = static_cast<int>(ii * SE3::DOF);

    error_block.segment<SE3::DOF>(offset) =
        (pose.observation_from_scene * point.point.frame_from_ref().inverse())
            .log();

    const int prev_idx = idx(prev_it);
    const int next_idx = idx(next_it);
    for (int jj = 0U; jj < SE3::DOF; ++jj) {
      for (int kk = 0U; kk < TWO_JET_DOF<SE3>; ++kk) {
        error_jacobian_writer(
            key_,
            offset + jj,
            prev_idx * static_cast<int>(TWO_JET_DOF<SE3>) + kk,
            point.d_prev(jj, kk));
        error_jacobian_writer(
            key_,
            offset + jj,
            next_idx * TWO_JET_DOF<SE3> + kk,
            point.d_next(jj, kk));
      }
    }
  }
}

}  // namespace resim::curves::optimization
