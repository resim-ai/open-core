// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/actor/actor.hh"

#include "resim/utils/uuid.hh"

namespace resim::actor {

Actor::Actor(const ActorId id) : id_{id} {}

ActorId Actor::id() const { return id_; }

}  // namespace resim::actor
