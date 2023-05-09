#include "resim_core/experiences/proto/storyboard_to_proto.hh"

#include "resim_core/actor/state/proto/trajectory_to_proto.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/experiences/proto/actor.pb.h"
#include "resim_core/experiences/proto/ilqr_drone_to_proto.hh"
#include "resim_core/experiences/proto/storyboard.pb.h"
#include "resim_core/experiences/storyboard.hh"
#include "resim_core/utils/match.hh"
#include "resim_core/utils/proto/uuid_to_proto.hh"

namespace resim::experiences::proto {

void pack(const experiences::MovementModel &in, MovementModel *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  typename Actor::Reference *const actor_reference =
      out->mutable_actor_reference();
  pack(in.actor_reference, actor_reference->mutable_id());
  match(
      in.model,
      [&](const actor::state::Trajectory &trajectory) {
        pack(trajectory, out->mutable_trajectory_model());
      },
      [&](const experiences::ILQRDrone &ilqr_drone) {
        pack(ilqr_drone, out->mutable_ilqr_drone());
      });
}

void pack(const experiences::Storyboard &in, Storyboard *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  for (const experiences::MovementModel &model : in.movement_models) {
    pack(model, out->add_movement_models());
  }
}

experiences::MovementModel unpack(const MovementModel &in) {
  experiences::MovementModel result;
  result.actor_reference = unpack(in.actor_reference().id());
  if (in.has_trajectory_model()) {
    result.model = unpack(in.trajectory_model());
  } else if (in.has_ilqr_drone()) {
    result.model = unpack(in.ilqr_drone());
  } else {
    REASSERT(false, "MovementModel proto has no model!");
  }
  return result;
}

experiences::Storyboard unpack(const Storyboard &in) {
  experiences::Storyboard result;
  for (const MovementModel &model : in.movement_models()) {
    result.movement_models.push_back(unpack(model));
  }
  return result;
}

}  // namespace resim::experiences::proto
