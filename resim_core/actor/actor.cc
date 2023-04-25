
#include "resim_core/actor/actor.hh"

#include "resim_core/utils/uuid.hh"

namespace resim::actor {

Actor::Actor() : id_{UUID::new_uuid()} {}

ActorId Actor::id() const { return id_; }

}  // namespace resim::actor
