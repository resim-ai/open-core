#include "resim_core/experiences/proto/location_condition_to_proto.hh"

#include "resim_core/assert/assert.hh"
#include "resim_core/experiences/location_condition.hh"
#include "resim_core/experiences/proto/actor.pb.h"
#include "resim_core/experiences/proto/location_condition.pb.h"
#include "resim_core/transforms/proto/se3_to_proto.hh"
#include "resim_core/utils/proto/uuid_to_proto.hh"

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
