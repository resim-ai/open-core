
#include "resim/actor/actor.hh"

#include "resim/utils/uuid.hh"

namespace resim::actor {

Actor::Actor(const ActorId id) : id_{id} {}

ActorId Actor::id() const { return id_; }

}  // namespace resim::actor
