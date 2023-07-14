#pragma once

#include <vector>

#include "resim/actor/state/observable_state.hh"
#include "resim/actor/state/proto/observable_state.pb.h"

namespace resim::actor::state::proto {

void pack(const actor::state::ObservableState &in, ObservableState *out);

actor::state::ObservableState unpack(const ObservableState &in);

void pack(
    const std::vector<actor::state::ObservableState> &in,
    ObservableStates *out);

std::vector<actor::state::ObservableState> unpack(const ObservableStates &in);

}  // namespace resim::actor::state::proto
