// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/actor/state/proto/trajectory_to_proto.hh"

#include "google/protobuf/timestamp.pb.h"
#include "resim/actor/state/proto/trajectory.pb.h"
#include "resim/actor/state/trajectory.hh"
#include "resim/assert/assert.hh"
#include "resim/curves/proto/t_curve_se3_to_proto.hh"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/time/timestamp.hh"

namespace resim::actor::state::proto {

void pack(const state::Trajectory &in, Trajectory *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  time::proto::pack(in.start_time(), out->mutable_start_time());
  pack(in.curve(), out->mutable_curve());
}

state::Trajectory unpack(const Trajectory &in) {
  return state::Trajectory(
      unpack(in.curve()),
      time::proto::unpack(in.start_time()));
}

}  // namespace resim::actor::state::proto
