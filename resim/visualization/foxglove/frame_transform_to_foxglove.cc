// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <foxglove/FrameTransform.pb.h>

#include "resim/assert/assert.hh"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"
#include "resim/visualization/foxglove/orientation_to_foxglove.hh"
#include "resim/visualization/foxglove/vector_to_foxglove.hh"

namespace resim::visualization::foxglove {

void pack_into_foxglove(
    const transforms::SE3 &in,
    const time::Timestamp time,
    ::foxglove::FrameTransform *const out,
    const std::string &parent,
    const std::string &child) {
  REASSERT(out != nullptr, "Can't pack invalid frame transform!");
  REASSERT(in.is_framed(), "Can only pack framed SE3s to FrameTransforms!");
  out->Clear();
  time::proto::pack(time, out->mutable_timestamp());
  out->set_parent_frame_id(
      parent.empty() ? in.into().id().to_string() : parent);
  out->set_child_frame_id(child.empty() ? in.from().id().to_string() : child);
  pack_into_foxglove(in.translation(), out->mutable_translation());
  pack_into_foxglove(in.rotation(), out->mutable_rotation());
}

}  // namespace resim::visualization::foxglove
