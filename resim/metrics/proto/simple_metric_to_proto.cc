// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/metrics/proto/simple_metric_to_proto.hh"

#include <optional>

#include "google/protobuf/timestamp.pb.h"
#include "resim/actor/actor_id.hh"
#include "resim/assert/assert.hh"
#include "resim/metrics/proto/simple_metric.pb.h"
#include "resim/metrics/simple_metric.hh"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/proto/uuid_to_proto.hh"
namespace resim::metrics::proto {

void pack(
    const metrics::SimpleMetric &in,
    metrics::proto::SimpleMetric *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  time::proto::pack(in.time, out->mutable_time());
  out->set_name(in.name);

  if (in.actor_id.has_value()) {
    resim::proto::pack(in.actor_id.value(), out->mutable_actor_id());
  }

  if (in.metric_value.has_value()) {
    out->set_metric_value(in.metric_value.value());
  }
}

metrics::SimpleMetric unpack(const metrics::proto::SimpleMetric &in) {
  return metrics::SimpleMetric(
      in.name(),
      time::proto::unpack(in.time()),
      in.has_actor_id() ? std::make_optional(unpack(in.actor_id()))
                        : std::nullopt,
      in.has_metric_value() ? std::make_optional<double>(in.metric_value())
                            : std::nullopt);
}

}  // namespace resim::metrics::proto
