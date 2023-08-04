// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/experiences/proto/actor_to_proto.hh"

#include "resim/assert/assert.hh"
#include "resim/experiences/actor.hh"
#include "resim/experiences/proto/actor.pb.h"
#include "resim/utils/proto/uuid_to_proto.hh"

namespace resim::experiences::proto {

void pack(const experiences::Actor &in, Actor *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  pack(in.id, out->mutable_id());
  out->set_name(in.name);
  switch (in.actor_type) {
    case ActorType::SIMULATION_ACTOR:
      out->set_actor_type(Actor_ActorType_SIMULATION_ACTOR);
      break;
    case ActorType::SYSTEM_UNDER_TEST:
      out->set_actor_type(Actor_ActorType_SYSTEM_UNDER_TEST);
      break;
    default:
      // This should never happen
      REASSERT(false);
      out->set_actor_type(Actor_ActorType_INVALID);
  }
  for (const auto &geometry : in.geometries) {
    pack(geometry.geometry_id, out->add_geometries()->mutable_geometry_id());
  }
}

experiences::Actor unpack(const Actor &in) {
  experiences::Actor result;
  result.id = unpack(in.id());
  result.name = in.name();
  switch (in.actor_type()) {
    case Actor_ActorType_SIMULATION_ACTOR:
      result.actor_type = ActorType::SIMULATION_ACTOR;
      break;
    case Actor_ActorType_SYSTEM_UNDER_TEST:
      result.actor_type = ActorType::SYSTEM_UNDER_TEST;
      break;
    default:
      // This should never happen
      REASSERT(false);
      result.actor_type = ActorType::INVALID;
  }
  result.geometries.reserve(in.geometries_size());
  for (const auto &geometry_msg : in.geometries()) {
    result.geometries.push_back({
        .geometry_id = unpack(geometry_msg.geometry_id()),
    });
  }
  return result;
}

}  // namespace resim::experiences::proto
