// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/experiences/proto/ilqr_drone_to_proto.hh"

#include "resim/assert/assert.hh"
#include "resim/experiences/ilqr_drone.hh"
#include "resim/experiences/proto/ilqr_drone.pb.h"
#include "resim/math/proto/matrix_to_proto.hh"
#include "resim/utils/inout.hh"

namespace resim::experiences::proto {

void pack(const experiences::ILQRDrone &in, ILQRDrone *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  out->set_velocity_cost(in.velocity_cost);
  math::proto::pack_matrix(
      in.initial_position,
      out->mutable_initial_position());
  math::proto::pack_matrix(in.goal_position, out->mutable_goal_position());
}

experiences::ILQRDrone unpack(const ILQRDrone &in) {
  experiences::ILQRDrone result{
      .velocity_cost = in.velocity_cost(),
  };
  math::proto::unpack_matrix(
      in.initial_position(),
      InOut{result.initial_position});
  math::proto::unpack_matrix(in.goal_position(), InOut{result.goal_position});
  return result;
}

}  // namespace resim::experiences::proto
