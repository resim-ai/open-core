#include "resim_core/metrics/min_distance.hh"

#include <algorithm>
#include <limits>
#include <numeric>

#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/transforms/framed_group.hh"

namespace resim::metrics {

std::optional<double> min_distance(
    const actor::state::ObservableState& target_state,
    const std::vector<actor::state::ObservableState>& states) {
  if (!target_state.is_spawned) {
    return std::nullopt;
  }

  return std::accumulate(
      states.begin(),
      states.end(),
      std::optional<double>(std::nullopt),
      [target_state](
          std::optional<double> curr_min_distance,
          const actor::state::ObservableState& other_state) {
        REASSERT(
            target_state.time_of_validity == other_state.time_of_validity,
            "Time mismatch in actor states");
        if ((target_state.id == other_state.id || !other_state.is_spawned)) {
          return curr_min_distance;
        }

        const double other_distance = transforms::fse3_inverse_distance(
            target_state.state.ref_from_body(),
            other_state.state.ref_from_body());

        if (!curr_min_distance.has_value()) {
          return std::make_optional<double>(other_distance);
        }

        return std::make_optional<double>(
            std::min(other_distance, curr_min_distance.value()));
      });
}

std::optional<double> min_distance(
    const actor::ActorId& target_actor_id,
    const std::vector<actor::state::ObservableState>& states) {
  auto target_state = std::find_if(
      states.begin(),
      states.end(),
      [&target_actor_id](const actor::state::ObservableState& state) {
        return state.id == target_actor_id;
      });

  REASSERT(target_state != states.end(), "Target ID not found in list");
  return min_distance(*target_state, states);
}

}  // namespace resim::metrics
