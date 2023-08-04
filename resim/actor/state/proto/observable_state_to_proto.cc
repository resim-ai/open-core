// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#include "resim/actor/state/proto/observable_state_to_proto.hh"

#include "resim/actor/state/observable_state.hh"
#include "resim/actor/state/rigid_body_state.hh"
#include "resim/assert/assert.hh"
#include "resim/curves/proto/two_jetr_se3_to_proto.hh"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/proto/uuid_to_proto.hh"

namespace resim::actor::state::proto {

void pack(const actor::state::ObservableState &in, ObservableState *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid observable state message!");
  out->Clear();
  pack(in.id, out->mutable_id());
  out->set_is_spawned(in.is_spawned);

  time::proto::pack(in.time_of_validity, out->mutable_time_of_validity());

  pack(in.state.ref_from_body_two_jet(), out->mutable_state());
}

actor::state::ObservableState unpack(const ObservableState &in) {
  return actor::state::ObservableState{
      .id = unpack(in.id()),
      .is_spawned = in.is_spawned(),
      .time_of_validity = time::proto::unpack(in.time_of_validity()),
      .state = RigidBodyState<transforms::SE3>{unpack(in.state())},
  };
}

void pack(
    const std::vector<actor::state::ObservableState> &in,
    ObservableStates *const out) {
  REASSERT(
      out != nullptr,
      "Can't pack into invalid observable states message!");
  out->Clear();
  for (const actor::state::ObservableState &state : in) {
    pack(state, out->add_states());
  }
}

std::vector<actor::state::ObservableState> unpack(const ObservableStates &in) {
  std::vector<actor::state::ObservableState> result;
  result.reserve(in.states_size());
  for (const ObservableState &state : in.states()) {
    result.push_back(unpack(state));
  }
  return result;
}

}  // namespace resim::actor::state::proto
