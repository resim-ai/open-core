#include "resim_core/metrics/proto/simple_metric_to_proto.hh"

#include <optional>

#include "google/protobuf/timestamp.pb.h"
#include "resim_core/actor/actor_id.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/metrics/proto/simple_metric.pb.h"
#include "resim_core/metrics/simple_metric.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/utils/proto/uuid_to_proto.hh"

namespace resim::metrics::proto {

void pack(
    const metrics::SimpleMetric &in,
    metrics::proto::SimpleMetric *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  google::protobuf::Timestamp timestamp;
  time::SecsAndNanos time =
      time::to_seconds_and_nanos(in.time.time_since_epoch());
  out->mutable_time()->set_nanos(time.nanos);
  out->mutable_time()->set_seconds(time.secs);
  out->set_name(in.name);

  if (in.actor_id.has_value()) {
    resim::proto::pack(in.actor_id.value(), out->mutable_actor_id());
  }

  if (in.metric_value.has_value()) {
    out->set_metric_value(in.metric_value.value());
  }
}

metrics::SimpleMetric unpack(const metrics::proto::SimpleMetric &in) {
  time::Timestamp time{
      time::from_seconds_and_nanos({in.time().seconds(), in.time().nanos()})};

  return metrics::SimpleMetric(
      in.name(),
      time,
      in.has_actor_id() ? std::make_optional(unpack(in.actor_id()))
                        : std::nullopt,
      in.has_metric_value() ? std::make_optional<double>(in.metric_value())
                            : std::nullopt);
}

}  // namespace resim::metrics::proto
