// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/foxglove/orientation_to_foxglove.hh"

#include <Eigen/Dense>

#include "resim/assert/assert.hh"

namespace resim::visualization::foxglove {

void pack_into_foxglove(
    const transforms::SO3 &in,
    ::foxglove::Quaternion *const out) {
  REASSERT(out != nullptr, "Can't pack invalid orientation!");
  out->Clear();
  const Eigen::Quaterniond quaternion{in.quaternion()};
  out->set_w(quaternion.w());
  out->set_x(quaternion.x());
  out->set_y(quaternion.y());
  out->set_z(quaternion.z());
}

}  // namespace resim::visualization::foxglove
