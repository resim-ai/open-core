#pragma once

#include <string>
#include <vector>

#include "resim_core/time/timestamp.hh"

namespace resim::metrics {

// This class is used for representing simple generic continuous metrics which
// we want to log to foxglove, defined by:
// - name: A name for the metric, which should likely correspond to channel name
// - time: A timestamp for the metric (which may *not* correspond to publish
// time, but will normally correspond to publish time)
// - value: A perhaps empty double vector, storing the metric value at the time
//
// A line chart can be represented by a list of SimpleMetrics published to one
// channel, with only one double stored in each value vector
struct SimpleMetric {
  std::string name;
  time::Timestamp time;
  std::vector<double> value;

  SimpleMetric() = default;
  SimpleMetric(std::string n, time::Timestamp t, std::vector<double> v)
      : name(std::move(n)),
        time(t),
        value(std::move(v)){};

  bool operator==(const SimpleMetric&) const = default;
};

}  // namespace resim::metrics
