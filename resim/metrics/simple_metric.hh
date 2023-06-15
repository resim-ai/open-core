#pragma once

#include <optional>
#include <string>

#include "resim/actor/actor_id.hh"
#include "resim/time/timestamp.hh"

namespace resim::metrics {

// This class is used for representing simple generic continuous metrics which
// we want to log to foxglove, defined by:
// - name: A name for the metric, which should likely correspond to channel name
// - time: A timestamp for the metric (which may *not* correspond to publish
// time, but will normally correspond to publish time)
// - metric_value: A perhaps empty double, storing the metric value at the time
//
// A line chart can be represented by a list of SimpleMetrics published to one
// channel, with only one double stored in each value vector
struct SimpleMetric {
  std::string name;
  time::Timestamp time;
  std::optional<actor::ActorId> actor_id;

  std::optional<double> metric_value;

  SimpleMetric() = default;
  SimpleMetric(
      std::string n,
      time::Timestamp t,
      std::optional<actor::ActorId> id,
      std::optional<double> v)
      : name(std::move(n)),
        time(t),
        actor_id(id),
        metric_value(v){};

  bool operator==(const SimpleMetric&) const = default;
};

}  // namespace resim::metrics
