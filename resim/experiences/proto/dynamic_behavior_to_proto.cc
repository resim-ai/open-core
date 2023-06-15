#include "resim/experiences/proto/dynamic_behavior_to_proto.hh"

#include "resim/assert/assert.hh"
#include "resim/experiences/dynamic_behavior.hh"
#include "resim/experiences/proto/actor_to_proto.hh"
#include "resim/experiences/proto/completion_criteria_to_proto.hh"
#include "resim/experiences/proto/dynamic_behavior.pb.h"
#include "resim/experiences/proto/storyboard_to_proto.hh"

namespace resim::experiences::proto {

void pack(const experiences::DynamicBehavior &in, DynamicBehavior *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  for (const experiences::Actor &actor : in.actors) {
    pack(actor, out->add_actors());
  }
  pack(in.storyboard, out->mutable_storyboard());
  pack(in.completion_criteria, out->mutable_completion_criteria());
}

experiences::DynamicBehavior unpack(const DynamicBehavior &in) {
  experiences::DynamicBehavior result;
  result.storyboard = unpack(in.storyboard());
  result.completion_criteria = unpack(in.completion_criteria());
  for (const Actor &actor : in.actors()) {
    result.actors.push_back(unpack(actor));
  }
  return result;
}

}  // namespace resim::experiences::proto
