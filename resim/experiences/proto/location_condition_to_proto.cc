// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/experiences/proto/location_condition_to_proto.hh"

#include "resim/assert/assert.hh"
#include "resim/experiences/location_condition.hh"
#include "resim/experiences/proto/actor.pb.h"
#include "resim/experiences/proto/location_condition.pb.h"
#include "resim/transforms/proto/se3_to_proto.hh"
#include "resim/utils/proto/uuid_to_proto.hh"

namespace resim::experiences::proto {

void pack(
    const experiences::LocationCondition &in,
    LocationCondition *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  pack(in.target_position, out->mutable_target_position());
  out->set_tolerance_m(in.tolerance_m);
  typename Actor::Reference *const triggering_actor =
      out->mutable_triggering_actor();
  pack(in.triggering_actor, triggering_actor->mutable_id());
}

experiences::LocationCondition unpack(const LocationCondition &in) {
  experiences::LocationCondition result;
  result.target_position = unpack(in.target_position());
  result.tolerance_m = in.tolerance_m();
  result.triggering_actor = unpack(in.triggering_actor().id());
  return result;
}

}  // namespace resim::experiences::proto
