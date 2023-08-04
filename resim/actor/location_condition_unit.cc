// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/actor/location_condition_unit.hh"

#include <memory>
#include <unordered_map>
#include <vector>

#include "resim/actor/state/observable_state.hh"
#include "resim/assert/assert.hh"
#include "resim/experiences/completion_criteria.hh"
#include "resim/simulator/executor_builder.hh"
#include "resim/simulator/simulation_unit.hh"
#include "resim/simulator/standard_topics.hh"
#include "resim/simulator/time_lord_update.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/inout.hh"

namespace resim::actor {
LocationConditionUnit::LocationConditionUnit(
    std::shared_ptr<LoggerInterface> logger_interface,
    const experiences::CompletionCriteria &criteria,
    InOut<simulator::ExecutorBuilder> executor_builder)
    : SimulationUnit(std::move(logger_interface)) {
  // This must be local, as captured by a lambda.
  std::unordered_multimap<ActorId, experiences::Condition> id_to_conditions{};

  for (const experiences::Condition &condition : criteria.conditions) {
    REASSERT(condition.delay >= time::Duration(std::chrono::nanoseconds(0)));
    REASSERT(condition.condition.tolerance_m >= 0.0);
    id_to_conditions.emplace(condition.condition.triggering_actor, condition);
  }

  executor_builder->add_task<
      state::ObservableState,
      std::vector<simulator::TimeLordUpdate>>(
      "location_condition",
      simulator::ACTOR_STATES_TOPIC,
      simulator::SCHEDULE_TIMELORD_TOPIC,
      [id_to_conditions = std::move(id_to_conditions)](
          const std::vector<state::ObservableState> &states) {
        std::vector<simulator::TimeLordUpdate> terminations{};
        for (const state::ObservableState &state : states) {
          auto [begin, end] = id_to_conditions.equal_range(state.id);
          for (auto iter = begin; iter != end; ++iter) {
            const experiences::Condition &condition = iter->second;
            if (transforms::se3_inverse_distance(
                    condition.condition.target_position,
                    state.state.ref_from_body()) <=
                condition.condition.tolerance_m) {
              // TODO(tknowles): Add some logging here.
              terminations.emplace_back(
                  state.time_of_validity + condition.delay,
                  simulator::TimeLordUpdate::UpdateBehaviour::
                      FULL_UPDATE_THEN_TERMINATE,
                  simulator::TimeLordUpdate::UpdateType::UNIT_SCHEDULED);
            }
          }
        }
        return terminations;
      });
};
}  // namespace resim::actor
