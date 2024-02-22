// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/foxglove/pose_to_foxglove.hh"

#include "resim/assert/assert.hh"
#include "resim/visualization/foxglove/orientation_to_foxglove.hh"
#include "resim/visualization/foxglove/vector_to_foxglove.hh"

namespace resim::visualization::foxglove {

void pack_into_foxglove(
    const transforms::SE3 &in,
    ::foxglove::Pose *const out) {
  REASSERT(out != nullptr, "Can't pack invalid pose!");
  out->Clear();
  pack_into_foxglove(in.rotation(), out->mutable_orientation());
  pack_into_foxglove(in.translation(), out->mutable_position());
}

}  // namespace resim::visualization::foxglove
