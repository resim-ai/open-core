#pragma once

#include <optional>
#include <vector>

#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/state/observable_state.hh"

namespace resim::metrics {

// Computes the minimum distance between a target actor, and a list of
// actors. If an actor with the target ID is present in  the list, it is
// functionally ignored. Unspawned actors are also ignored.
// Returns std::nullopt if no (unignored) actors are found.
std::optional<double> min_distance(
    const actor::state::ObservableState& target_state,
    const std::vector<actor::state::ObservableState>& states);

// Computes the minimum distance between a target actor ID found in a list of
// actors, and the rest of the actors (ignoring the actor itself.)
// Unspawned actors are also ignored.
// Returns std::nullopt if no (unignored) actors are found.
std::optional<double> min_distance(
    const actor::ActorId& target_actor_id,
    const std::vector<actor::state::ObservableState>& states);

}  // namespace resim::metrics
