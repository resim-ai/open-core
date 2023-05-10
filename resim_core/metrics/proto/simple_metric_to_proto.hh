#include "resim_core/metrics/proto/simple_metric.pb.h"
#include "resim_core/metrics/simple_metric.hh"

namespace resim::metrics::proto {

void pack(const metrics::SimpleMetric &in, metrics::proto::SimpleMetric *out);

metrics::SimpleMetric unpack(const metrics::proto::SimpleMetric &in);

}  // namespace resim::metrics::proto
