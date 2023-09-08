// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/metrics/proto/validate_metrics_proto.hh"

#include <exception>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "google/protobuf/timestamp.pb.h"
#include "resim/actor/actor_id.hh"
#include "resim/assert/assert.hh"
#include "resim/metrics/proto/metrics.pb.h"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/proto/uuid_to_proto.hh"
#include "resim/utils/uuid.hh"

namespace resim::metrics::proto {

void validate_double_summary_values_proto(
    const DoubleSummaryMetricValues& metric_values,
    const MetricsDataMap& map) {
  // TODO(tknowles): This validation is incomplete for the full features set of
  // double summaries
  REASSERT(metric_values.has_value_data_id());
  REASSERT(metric_values.has_status_data_id());
  resim::UUID id =
      resim::proto::unpack(metric_values.value_data_id().data_id());
  REASSERT(
      map.at(id).data_type() == INDEXED_DOUBLE_ARRAY_DATA_TYPE ||
      map.at(id).data_type() == DOUBLE_ARRAY_DATA_TYPE);
  resim::UUID status_id =
      resim::proto::unpack(metric_values.status_data_id().data_id());

  if (map.at(id).data_type() == INDEXED_DOUBLE_ARRAY_DATA_TYPE) {
    REASSERT(
        map.at(status_id).data_type() == INDEXED_METRIC_STATUS_ARRAY_DATA_TYPE);
    REASSERT(
        map.at(status_id).array().indexed_statuses().index_type() ==
        map.at(id).array().indexed_doubles().index_type());
    REASSERT(
        resim::proto::unpack(
            map.at(id).array().indexed_doubles().index_id().data_id()) ==
        resim::proto::unpack(
            map.at(status_id).array().indexed_statuses().index_id().data_id()));
  } else {
    REASSERT(map.at(status_id).data_type() == METRIC_STATUS_ARRAY_DATA_TYPE);
    REASSERT(map.at(status_id).length() == 1);
    REASSERT(map.at(id).length() == 1);
  }
}

void validate_double_over_time_metric_values_proto(
    const DoubleOverTimeMetricValues& metric_values,
    const MetricsDataMap& map) {
  REASSERT(metric_values.double_timestamps_data_id_size() >= 1);
  for (auto iter = metric_values.double_timestamps_data_id().begin();
       iter < metric_values.double_timestamps_data_id().end();
       ++iter) {
    resim::UUID id = resim::proto::unpack(iter->data_id());
    REASSERT(map.at(id).data_type() == INDEXED_DOUBLE_ARRAY_DATA_TYPE);
    REASSERT(
        map.at(id).array().indexed_doubles().index_type() ==
        TIMESTAMP_ARRAY_DATA_TYPE);
  }
}

void validate_line_plot_metric_values_proto(
    const LinePlotMetricValues& metric_values,
    const MetricsDataMap& map) {
  REASSERT(metric_values.x_doubles_data_id_size() >= 1);
  REASSERT(
      metric_values.y_doubles_data_id_size() ==
      metric_values.x_doubles_data_id_size());
  for (int i = 0; i < metric_values.x_doubles_data_id_size(); ++i) {
    resim::UUID x_id =
        resim::proto::unpack(metric_values.x_doubles_data_id(i).data_id());
    resim::UUID y_id =
        resim::proto::unpack(metric_values.y_doubles_data_id(i).data_id());
    REASSERT(map.at(x_id).data_type() == DOUBLE_ARRAY_DATA_TYPE);
    REASSERT(map.at(y_id).data_type() == DOUBLE_ARRAY_DATA_TYPE);
  }
}

void validate_bar_chart_metric_values_proto(
    const BarChartMetricValues& metric_values,
    const MetricsDataMap& map) {
  REASSERT(metric_values.values_data_id_size() >= 1);
  REASSERT(
      metric_values.statuses_data_id_size() ==
      metric_values.values_data_id_size());

  if (metric_values.stack_labels_size() > 0) {
    REASSERT(
        metric_values.stack_labels_size() ==
        metric_values.values_data_id_size());
  }

  for (int i = 0; i < metric_values.values_data_id_size(); ++i) {
    resim::UUID values_id =
        resim::proto::unpack(metric_values.values_data_id(i).data_id());
    resim::UUID statuses_id =
        resim::proto::unpack(metric_values.statuses_data_id(i).data_id());
    REASSERT(map.at(values_id).data_type() == DOUBLE_ARRAY_DATA_TYPE);
    REASSERT(map.at(statuses_id).data_type() == METRIC_STATUS_ARRAY_DATA_TYPE);
    REASSERT(map.at(values_id).length() == map.at(statuses_id).length());
    if (metric_values.axis_labels_size() > 0) {
      REASSERT(map.at(values_id).length() == metric_values.axis_labels_size());
    }
  }
}

void validate_states_over_time_metric_values_proto(
    const StatesOverTimeMetricValues& metric_values,
    const MetricsDataMap& map) {
  REASSERT(metric_values.state_timestamps_data_id_size() >= 1);
  REASSERT(
      metric_values.status_timestamps_data_id_size() ==
      metric_values.state_timestamps_data_id_size());

  std::unordered_set<std::string> states{};

  for (auto iter = metric_values.states_set().begin();
       iter < metric_values.states_set().end();
       ++iter) {
    states.insert(*iter);
  }

  for (int i = 0; i < metric_values.state_timestamps_data_id_size(); ++i) {
    resim::UUID state_id = resim::proto::unpack(
        metric_values.state_timestamps_data_id(i).data_id());
    resim::UUID status_id = resim::proto::unpack(
        metric_values.status_timestamps_data_id(i).data_id());

    REASSERT(map.at(state_id).data_type() == INDEXED_STRING_ARRAY_DATA_TYPE);
    REASSERT(
        map.at(state_id).array().indexed_strings().index_type() ==
        TIMESTAMP_ARRAY_DATA_TYPE);

    REASSERT(
        map.at(status_id).data_type() == INDEXED_METRIC_STATUS_ARRAY_DATA_TYPE);
    REASSERT(map.at(state_id).length() == map.at(status_id).length());
    REASSERT(
        resim::proto::unpack(map.at(status_id)
                                 .array()
                                 .indexed_statuses()
                                 .index_id()
                                 .data_id()) ==
        resim::proto::unpack(
            map.at(state_id).array().indexed_strings().index_id().data_id()));

    for (auto iter =
             map.at(state_id).array().indexed_strings().strings().begin();
         iter < map.at(state_id).array().indexed_strings().strings().end();
         ++iter) {
      REASSERT(states.contains(*iter));
    }
  }

  for (auto iter = metric_values.failure_states().begin();
       iter < metric_values.failure_states().end();
       ++iter) {
    REASSERT(states.contains(*iter));
  }
}

void validate_metric_proto(const Metric& metric, const MetricsDataMap& map) {
  switch (metric.type()) {
    case NO_METRIC_TYPE:
      REASSERT(false, "No metric type");
      break;
    case DOUBLE_SUMMARY:
      REASSERT(metric.metric_values().has_double_metric_values());
      validate_double_summary_values_proto(
          metric.metric_values().double_metric_values(),
          map);
      break;
    case DOUBLE_OVER_TIME:
      REASSERT(
          metric.metric_values().has_double_over_time_metric_values(),
          metric.name());
      validate_double_over_time_metric_values_proto(
          metric.metric_values().double_over_time_metric_values(),
          map);
      break;
    case LINE_PLOT:
      REASSERT(metric.metric_values().has_line_plot_metric_values());
      validate_line_plot_metric_values_proto(
          metric.metric_values().line_plot_metric_values(),
          map);
      break;
    case BAR_CHART:
      REASSERT(metric.metric_values().has_bar_chart_metric_values());
      validate_bar_chart_metric_values_proto(
          metric.metric_values().bar_chart_metric_values(),
          map);
      break;
    case STATES_OVER_TIME:
      REASSERT(metric.metric_values().has_states_over_time_metric_values());
      validate_states_over_time_metric_values_proto(
          metric.metric_values().states_over_time_metric_values(),
          map);
      break;
    default:
      REASSERT(false, "Invalid metric type");
      break;
  }
}

void validate_metrics_collection_proto(
    const MetricCollection& metric_collection,
    const MetricsDataMap& map) {
  std::unordered_map<resim::UUID, Metric> all_ids{};
  for (auto iter = metric_collection.metrics().begin();
       iter < metric_collection.metrics().end();
       ++iter) {
    REASSERT(iter->has_id());
    REASSERT(iter->has_metric_values());
    REASSERT(iter->status() != NO_METRIC_STATUS);
    all_ids.insert({resim::proto::unpack(iter->id().metric_id()), *iter});
  }

  for (auto iter = metric_collection.failed_metrics().begin();
       iter < metric_collection.failed_metrics().end();
       ++iter) {
    resim::UUID uuid = resim::proto::unpack(iter->metric_id());
    REASSERT(all_ids.contains(uuid));
    REASSERT(all_ids.at(uuid).status() == FAILED);
  }

  for (auto iter = metric_collection.metrics().begin();
       iter < metric_collection.metrics().end();
       ++iter) {
    validate_metric_proto(*iter, map);
  }
}

void validate_array_proto(
    const Array& array,
    const MetricsDataType data_type,
    unsigned int length,
    const MetricsDataMap& map) {
  switch (data_type) {
    case NO_DATA_TYPE:
      REASSERT(false, "No data type");
      break;
    case DOUBLE_ARRAY_DATA_TYPE:
      REASSERT(array.has_doubles());
      REASSERT(array.doubles().doubles_size() == length);
      break;
    case TIMESTAMP_ARRAY_DATA_TYPE:
      REASSERT(array.has_timestamps());
      REASSERT(array.timestamps().timestamps_size() == length);
      break;
    case ACTOR_ID_ARRAY_DATA_TYPE:
      REASSERT(array.has_actor_ids());
      REASSERT(array.actor_ids().actor_ids_size() == length);
      break;
    case STRING_ARRAY_DATA_TYPE:
      REASSERT(array.has_strings());
      REASSERT(array.strings().strings_size() == length);
      break;
    case METRIC_STATUS_ARRAY_DATA_TYPE:
      REASSERT(array.has_statuses());
      REASSERT(array.statuses().statuses_size() == length);
      break;
    case INDEXED_DOUBLE_ARRAY_DATA_TYPE:
      REASSERT(array.has_indexed_doubles());
      REASSERT(array.indexed_doubles().doubles_size() == length);
      REASSERT(array.indexed_doubles().has_index_id());
      REASSERT(
          map.at(resim::proto::unpack(
                     array.indexed_doubles().index_id().data_id()))
              .data_type() == array.indexed_doubles().index_type());
      REASSERT(
          map.at(resim::proto::unpack(
                     array.indexed_doubles().index_id().data_id()))
              .length() == length);
      break;
    case INDEXED_TIMESTAMP_ARRAY_DATA_TYPE:
      REASSERT(array.has_indexed_timestamps());
      REASSERT(array.indexed_timestamps().timestamps_size() == length);
      REASSERT(array.indexed_timestamps().has_index_id());
      REASSERT(
          map.at(resim::proto::unpack(
                     array.indexed_timestamps().index_id().data_id()))
              .data_type() == array.indexed_timestamps().index_type());
      REASSERT(
          map.at(resim::proto::unpack(
                     array.indexed_timestamps().index_id().data_id()))
              .length() == length);
      break;
    case INDEXED_ACTOR_ID_ARRAY_DATA_TYPE:
      REASSERT(array.has_indexed_actor_ids());
      REASSERT(array.indexed_actor_ids().actor_ids_size() == length);
      REASSERT(array.indexed_actor_ids().has_index_id());
      REASSERT(
          map.at(resim::proto::unpack(
                     array.indexed_actor_ids().index_id().data_id()))
              .data_type() == array.indexed_actor_ids().index_type());
      REASSERT(
          map.at(resim::proto::unpack(
                     array.indexed_actor_ids().index_id().data_id()))
              .length() == length);
      break;
    case INDEXED_STRING_ARRAY_DATA_TYPE:
      REASSERT(array.has_indexed_strings());
      REASSERT(array.indexed_strings().strings_size() == length);
      REASSERT(array.indexed_strings().has_index_id());
      REASSERT(
          map.at(resim::proto::unpack(
                     array.indexed_strings().index_id().data_id()))
              .data_type() == array.indexed_strings().index_type());
      REASSERT(
          map.at(resim::proto::unpack(
                     array.indexed_strings().index_id().data_id()))
              .length() == length);
      break;
    case INDEXED_METRIC_STATUS_ARRAY_DATA_TYPE:
      REASSERT(array.has_indexed_statuses());
      REASSERT(array.indexed_statuses().statuses_size() == length);
      REASSERT(array.indexed_statuses().has_index_id());
      REASSERT(
          map.at(resim::proto::unpack(
                     array.indexed_statuses().index_id().data_id()))
              .data_type() == array.indexed_statuses().index_type());
      REASSERT(
          map.at(resim::proto::unpack(
                     array.indexed_statuses().index_id().data_id()))
              .length() == length);
      break;
    default:
      REASSERT(false, "Invalid data type");
      break;
  }
}

void validate_metrics_data_proto(
    const MetricsData& data,
    const MetricsDataMap& map) {
  REASSERT(data.has_id());
  if (data.is_per_actor()) {
    REASSERT(data.has_per_actor_data());
    REASSERT(not data.has_array());
    REASSERT(data.actors_size() == data.per_actor_data().actor_data_size());
    for (auto iter = data.per_actor_data().actor_data().begin();
         iter < data.per_actor_data().actor_data().end();
         ++iter) {
      REASSERT(iter->has_id());
      validate_array_proto(iter->array(), data.data_type(), data.length(), map);
    }
  } else {
    REASSERT(not data.has_per_actor_data());
    REASSERT(data.has_array());
    validate_array_proto(data.array(), data.data_type(), data.length(), map);
  }
}

void validate_job_metrics_collection_proto(
    const MetricCollection& job_metrics_collection,
    const MetricsDataMap& map) {
  REASSERT(job_metrics_collection.metrics_status() != NO_METRIC_STATUS);
  for (auto iter = job_metrics_collection.metrics().begin();
       iter < job_metrics_collection.metrics().end();
       ++iter) {
    REASSERT(iter->has_job_id());
  }

  validate_metrics_collection_proto(job_metrics_collection, map);
}

void validate_job_metrics_proto(const JobMetrics& job_metrics_msg) {
  validate_job_metrics_proto(
      job_metrics_msg,
      build_metrics_data_map(job_metrics_msg.metrics_data()));
}

void validate_job_metrics_proto(
    const JobMetrics& job_metrics_msg,
    const MetricsDataMap& map) {
  REASSERT(job_metrics_msg.metrics_status() != NO_METRIC_STATUS);
  REASSERT(job_metrics_msg.has_job_id());
  REASSERT(job_metrics_msg.has_job_metrics());

  for (auto iter = job_metrics_msg.metrics_data().begin();
       iter < job_metrics_msg.metrics_data().end();
       ++iter) {
    validate_metrics_data_proto(*iter, map);
  }

  const MetricCollection& job_metrics_collection =
      job_metrics_msg.job_metrics();

  validate_job_metrics_collection_proto(job_metrics_collection, map);
}

MetricsDataMap build_metrics_data_map(
    const google::protobuf::RepeatedPtrField<MetricsData> metrics_data) {
  MetricsDataMap map{};
  for (auto iter = metrics_data.begin(); iter < metrics_data.end(); ++iter) {
    REASSERT(iter->has_id());
    map.insert({resim::proto::unpack(iter->id().data_id()), *iter});
  }
  return map;
}

}  // namespace resim::metrics::proto
