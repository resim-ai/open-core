// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/transforms/proto/se3_to_proto.hh"

#include "resim/transforms/proto/liegroup_to_proto.hh"
#include "resim/transforms/proto/se3.pb.h"
#include "resim/transforms/se3.hh"

namespace resim::transforms::proto {

void pack(const transforms::SE3 &in, SE3 *const out) { pack_liegroup(in, out); }

transforms::SE3 unpack(const SE3 &in) {
  transforms::SE3 transform;
  unpack_liegroup(in, InOut(transform));
  return transform;
}

}  // namespace resim::transforms::proto
