// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/metrics/actor_metrics_unit.hh"

#include <fmt/core.h>

#include <optional>
#include <string>

#include "resim/actor/actor_id.hh"
#include "resim/assert/assert.hh"
#include "resim/metrics/min_distance.hh"
#include "resim/metrics/proto/simple_metric.pb.h"
#include "resim/metrics/proto/simple_metric_to_proto.hh"
#include "resim/metrics/simple_metric.hh"
#include "resim/simulator/standard_topics.hh"
#include "resim/utils/inout.hh"

namespace resim::metrics {

namespace {

// This is purely a logging topic, as opposed to a topic for unit
// communication, so is not registered in standard topics.
const std::string MIN_DISTANCE_TOPIC{"metric_min_distance"};

}  // namespace

ActorMetricsUnit::ActorMetricsUnit(
    std::shared_ptr<LoggerInterface> logger_interface,
    InOut<simulator::ExecutorBuilder> executor_builder,
    actor::ActorId actor_id)
    : SimulationUnit(std::move(logger_interface)),
      actor_id_(actor_id) {
  REASSERT(logger() != nullptr, "Null logger passed to metrics unit");
  executor_builder->add_task<actor::state::ObservableState>(
      fmt::format("log_actor_metrics_{}", actor_id.to_string()),
      simulator::ACTOR_STATES_TOPIC,
      simulator::NULL_TOPIC,
      [&](const std::vector<actor::state::ObservableState> &actor_states) {
        log_actor_metrics_(actor_states);
      });
}

void ActorMetricsUnit::log_actor_metrics_(
    const std::vector<actor::state::ObservableState> &actor_states) {
  // Min distance
  log_actor_min_distance_metric_(actor_states);

  // Further ego metrics to be added here.
}

void ActorMetricsUnit::log_actor_min_distance_metric_(
    const std::vector<actor::state::ObservableState> &actor_states) {
  // Min distance topic
  logger()->add_proto_channel<proto::SimpleMetric>(MIN_DISTANCE_TOPIC);

  REASSERT(!actor_states.empty());

  const std::optional<double> d = min_distance(actor_id_, actor_states);

  SimpleMetric min_distance_metric{};
  min_distance_metric.name = MIN_DISTANCE_TOPIC;
  min_distance_metric.time = actor_states.at(0).time_of_validity;
  min_distance_metric.actor_id = actor_id_;

  if (d.has_value()) {
    min_distance_metric.metric_value = d.value();
  }

  proto::SimpleMetric min_distance_msg;
  pack(min_distance_metric, &min_distance_msg);
  logger()->log_proto(
      MIN_DISTANCE_TOPIC,
      actor_states.at(0).time_of_validity,
      min_distance_msg);
}

}  // namespace resim::metrics
