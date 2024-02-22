// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/foxglove/vector_to_foxglove.hh"

#include "resim/assert/assert.hh"

namespace resim::visualization::foxglove {

void pack_into_foxglove(
    const Eigen::Vector3d &in,
    ::foxglove::Vector3 *const out) {
  REASSERT(out != nullptr, "Can't pack invalid vector!");
  out->Clear();
  out->set_x(in.x());
  out->set_y(in.y());
  out->set_z(in.z());
}

void pack_into_foxglove(
    const Eigen::Vector3d &in,
    ::foxglove::Point3 *const out) {
  REASSERT(out != nullptr, "Can't pack invalid point!");
  out->Clear();
  out->set_x(in.x());
  out->set_y(in.y());
  out->set_z(in.z());
}

}  // namespace resim::visualization::foxglove
