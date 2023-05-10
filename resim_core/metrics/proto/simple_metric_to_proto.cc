#include "resim_core/metrics/proto/simple_metric_to_proto.hh"

#include "google/protobuf/timestamp.pb.h"
#include "resim_core/assert/assert.hh"
#include "resim_core/metrics/proto/simple_metric.pb.h"
#include "resim_core/metrics/simple_metric.hh"
#include "resim_core/time/timestamp.hh"

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

  out->mutable_value()->Clear();
  out->mutable_value()->Add(in.value.begin(), in.value.end());
}

metrics::SimpleMetric unpack(const metrics::proto::SimpleMetric &in) {
  time::Timestamp time{
      time::from_seconds_and_nanos({in.time().seconds(), in.time().nanos()})};
  std::vector<double> value{in.value().begin(), in.value().end()};
  return metrics::SimpleMetric(in.name(), time, value);
}

}  // namespace resim::metrics::proto
