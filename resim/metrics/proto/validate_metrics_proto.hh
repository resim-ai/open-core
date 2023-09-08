// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <unordered_map>

#include "resim/metrics/proto/metrics.pb.h"
#include "resim/utils/uuid.hh"

namespace resim::metrics::proto {

using MetricsDataMap = std::unordered_map<resim::UUID, MetricsData>;

MetricsDataMap build_metrics_data_map(
    const google::protobuf::RepeatedPtrField<MetricsData> metrics_data);

void validate_job_metrics_proto(const JobMetrics& job_metrics_msg);

void validate_job_metrics_proto(
    const JobMetrics& job_metrics_msg,
    const MetricsDataMap& map);

void validate_metric_proto(const Metric& metric_msg, const MetricsDataMap& map);

void validate_metrics_data_proto(
    const MetricsData& metric_data_msg,
    const MetricsDataMap& map);

}  // namespace resim::metrics::proto
