# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import uuid

import resim.metrics.proto.metrics_pb2 as mp


class InvalidMetricsException(Exception):
    pass


def _metrics_assert(val: bool, msg: str = ''):
    if not val:
        raise InvalidMetricsException(msg)


def _validate_metrics_data_type(metrics_data_type: mp.MetricsDataType):
    _metrics_assert(metrics_data_type != mp.NO_DATA_TYPE)


def _validate_metric_status(metric_status: mp.MetricStatus):
    _metrics_assert(metric_status != mp.NO_METRIC_STATUS)


def _validate_metric_type(metric_type: mp.MetricType):
    _metrics_assert(metric_type != mp.NO_METRIC_TYPE)


def _validate_metric_importance(importance: mp.MetricImportance):
    _metrics_assert(importance != mp.NO_SPECIFIED_IMPORTANCE)


def _validate_uuid(unique_id):
    try:
        uuid.UUID(hex=unique_id.data)
    except ValueError:
        return InvalidMetricsException()


def _validate_metrics_data_id(metrics_data_id: mp.MetricsDataId):
    _validate_uuid(metrics_data_id.id)


def _validate_job_id(job_id: mp.JobId):
    _validate_uuid(job_id.id)


def _validate_metric_id(metric_id: mp.MetricId):
    _validate_uuid(metric_id.id)

_all_array_data_types = {
    mp.DOUBLE_ARRAY_DATA_TYPE,
    mp.TIMESTAMP_ARRAY_DATA_TYPE,
    mp.UUID_ARRAY_DATA_TYPE,
    mp.STRING_ARRAY_DATA_TYPE,
    mp.METRIC_STATUS_ARRAY_DATA_TYPE,
    mp.INDEXED_DOUBLE_ARRAY_DATA_TYPE,
    mp.INDEXED_TIMESTAMP_ARRAY_DATA_TYPE,
    mp.INDEXED_UUID_ARRAY_DATA_TYPE,
    mp.INDEXED_STRING_ARRAY_DATA_TYPE,
    mp.INDEXED_METRIC_STATUS_ARRAY_DATA_TYPE,
}


def is_indexed(data_type: mp.MetricsDataType):
    return data_type in (
        mp.INDEXED_DOUBLE_ARRAY_DATA_TYPE,
        mp.INDEXED_TIMESTAMP_ARRAY_DATA_TYPE,
        mp.INDEXED_UUID_ARRAY_DATA_TYPE,
        mp.INDEXED_STRING_ARRAY_DATA_TYPE,
        mp.INDEXED_METRIC_STATUS_ARRAY_DATA_TYPE)


def _array_length(array: mp.Array):
    _metrics_assert(array.WhichOneof("array") is not None)
    if array.HasField("doubles"):
        return len(array.doubles.array)
    elif array.HasField("timestamps"):
        return len(array.timestamps.array)
    elif array.HasField("uuids"):
        return len(array.uuids.array)
    elif array.HasField("strings"):
        return len(array.strings.array)
    else: # array.HasField("statuses"):
        return len(array.statuses.array)


def _validate_data_arrays_agree(data_a: mp.MetricsData,
                                data_b: mp.MetricsData):
    pass
    _metrics_assert(data_a.WhichOneof("data") is not None)
    _metrics_assert(data_a.WhichOneof("data") == data_b.WhichOneof("data"))
    if data_a.HasField("array"):
        _metrics_assert(
            _array_length(
                data_a.array) == _array_length(
                data_b.array))
    else: # data_a.HasField("array_per_category")
        data_a_arrays = data_a.array_per_category.category_to_array
        data_b_arrays = data_b.array_per_category.category_to_array

        _metrics_assert(len(data_a_arrays) == len(data_b_arrays))
        for key in data_a_arrays:
            _metrics_assert(key in data_b_arrays)
            _metrics_assert(
                _array_length(
                    data_a_arrays[key]) == _array_length(
                    data_b_arrays[key]))


def _validate_values_and_statuses(value_data_id: mp.MetricsDataId,
                                  status_data_id: mp.MetricsDataId,
                                  metrics_data_map: dict[str, mp.MetricsData],
                                  *,
                                  allowed_value_types: set[mp.MetricsDataType] = _all_array_data_types,
                                  allowed_index_types: set[mp.MetricsDataType] = _all_array_data_types
                                  ):
    # Check that the referenced values data is good
    _validate_metrics_data_id(value_data_id)
    id_str = value_data_id.id.data
    _metrics_assert(id_str in metrics_data_map)
    value_data = metrics_data_map[id_str]
    _metrics_assert(value_data.data_type in allowed_value_types)

    # Check that the referenced status data is good and matches the values
    _validate_metrics_data_id(status_data_id)
    id_str = status_data_id.id.data
    _metrics_assert(id_str in metrics_data_map)
    status_data = metrics_data_map[id_str]

    if not is_indexed(value_data.data_type):
        _metrics_assert(status_data.data_type ==
                        mp.METRIC_STATUS_ARRAY_DATA_TYPE)
    else:
        _metrics_assert(status_data.data_type ==
                        mp.INDEXED_METRIC_STATUS_ARRAY_DATA_TYPE)
        _metrics_assert(status_data.index_data_id == value_data.index_data_id)
        _metrics_assert(status_data.index_data_type in allowed_index_types)
        _metrics_assert(value_data.index_data_type in allowed_index_types)

    _validate_data_arrays_agree(value_data, status_data)


def _validate_double_metric_values(
        double_metric_values: mp.DoubleSummaryMetricValues, metrics_data_map: dict[str, mp.MetricsData]):

  
    if double_metric_values.HasField('timestamp_index'):
        allowed_value_types =  {mp.INDEXED_DOUBLE_ARRAY_DATA_TYPE}
        allowed_index_types = {mp.TIMESTAMP_ARRAY_DATA_TYPE,
                               mp.INDEXED_TIMESTAMP_ARRAY_DATA_TYPE}
    elif double_metric_values.HasField('uuid_index'):
        allowed_value_types =  {mp.INDEXED_DOUBLE_ARRAY_DATA_TYPE}
        allowed_index_types = {mp.UUID_ARRAY_DATA_TYPE,
                               mp.INDEXED_UUID_ARRAY_DATA_TYPE}
    elif double_metric_values.HasField('string_index'):
        allowed_value_types =  {mp.INDEXED_DOUBLE_ARRAY_DATA_TYPE}
        allowed_index_types = {mp.STRING_ARRAY_DATA_TYPE,
                               mp.INDEXED_STRING_ARRAY_DATA_TYPE}
    else:
        # array_index or no index
        allowed_value_types =  {mp.DOUBLE_ARRAY_DATA_TYPE,
                                      mp.INDEXED_DOUBLE_ARRAY_DATA_TYPE}
        allowed_index_types = _all_array_data_types

    _validate_values_and_statuses(double_metric_values.value_data_id,
                                  double_metric_values.status_data_id,
                                  metrics_data_map,
                                  allowed_value_types=allowed_value_types,
                                  allowed_index_types=allowed_index_types)

    # No constraints on failure_definition


def _validate_double_over_time_metric_values(
        double_over_time_metric_values: mp.DoubleOverTimeMetricValues, metrics_data_map: dict[str, mp.MetricsData]):
    length = len(double_over_time_metric_values.doubles_over_time_data_id)
    _metrics_assert(
        length == len(
            double_over_time_metric_values.statuses_over_time_data_id))
    _metrics_assert(
        length == len(
            double_over_time_metric_values.failure_definition))

    for ii in range(length):
        _validate_values_and_statuses(
            double_over_time_metric_values.doubles_over_time_data_id[ii],
            double_over_time_metric_values.statuses_over_time_data_id[ii],
            metrics_data_map,
            allowed_value_types={mp.INDEXED_DOUBLE_ARRAY_DATA_TYPE},
            allowed_index_types={
                mp.TIMESTAMP_ARRAY_DATA_TYPE,
                mp.INDEXED_TIMESTAMP_ARRAY_DATA_TYPE})

def _validate_line_plot_metric_values(
        line_plot_metric_values: mp.LinePlotMetricValues, metrics_data_map: dict[str, mp.MetricsData]):
    length = len(line_plot_metric_values.x_doubles_data_id)
    _metrics_assert(
        length == len(
            line_plot_metric_values.y_doubles_data_id))
    _metrics_assert(
        length == len(
            line_plot_metric_values.statuses_data_id))

    for ii in range(length):
        statuses_data_id = line_plot_metric_values.statuses_data_id[ii]
        for values_data_id in (line_plot_metric_values.x_doubles_data_id[ii],
                               line_plot_metric_values.y_doubles_data_id[ii]):
            _validate_values_and_statuses(
                values_data_id,
                statuses_data_id,
                metrics_data_map,
                allowed_value_types={mp.DOUBLE_ARRAY_DATA_TYPE, mp.INDEXED_DOUBLE_ARRAY_DATA_TYPE},
                allowed_index_types={
                    mp.TIMESTAMP_ARRAY_DATA_TYPE,
                    mp.INDEXED_TIMESTAMP_ARRAY_DATA_TYPE})


def _validate_bar_chart_metric_values(
        bar_chart_metric_values: mp.BarChartMetricValues, metrics_data_map: dict[str, mp.MetricsData]):
    length = len(bar_chart_metric_values.values_data_id)
    _metrics_assert(
        length == len(
            bar_chart_metric_values.statuses_data_id))

    for ii in range(length):
        _validate_values_and_statuses(
            bar_chart_metric_values.values_data_id[ii],
            bar_chart_metric_values.statuses_data_id[ii],
            metrics_data_map,
            allowed_value_types={mp.INDEXED_DOUBLE_ARRAY_DATA_TYPE},
            allowed_index_types={
                mp.STRING_ARRAY_DATA_TYPE,
                mp.INDEXED_STRING_ARRAY_DATA_TYPE})


def _validate_states_over_time_metric_values(
        states_over_time_metric_values: mp.StatesOverTimeMetricValues, metrics_data_map: dict[str, mp.MetricsData]):
    length = len(states_over_time_metric_values.states_over_time_data_id)
    _metrics_assert(
        length == len(
            states_over_time_metric_values.statuses_over_time_data_id))

    for ii in range(length):
        value_data_id = states_over_time_metric_values.states_over_time_data_id[ii]
        _validate_values_and_statuses(
            value_data_id,
            states_over_time_metric_values.statuses_over_time_data_id[ii],
            metrics_data_map,
            allowed_value_types={mp.INDEXED_STRING_ARRAY_DATA_TYPE},
            allowed_index_types={
                mp.TIMESTAMP_ARRAY_DATA_TYPE,
                mp.INDEXED_TIMESTAMP_ARRAY_DATA_TYPE})

        # Check that all the states are in the states set
        id_str = value_data_id.id.data
        value_data = metrics_data_map[id_str]
        if value_data.HasField('array'):
            _metrics_assert(value_data.array.HasField('strings'))
            for state in value_data.array.strings.array:
                 _metrics_assert(state in states_over_time_metric_values.states_set) 
        else:
            for category, array in value_data.array_per_category.category_to_array.items():
                _metrics_assert(array.HasField('strings'))
                for state in array.strings.array:
                    _metrics_assert(state in states_over_time_metric_values.states_set)

        _metrics_assert(set(states_over_time_metric_values.failure_states).issubset(states_over_time_metric_values.states_set))
    


def _validate_histogram_metric_values(
        histogram_metric_values: mp.HistogramMetricValues, metrics_data_map: dict[str, mp.MetricsData]):
    _validate_values_and_statuses(
        histogram_metric_values.values_data_id,
        histogram_metric_values.statuses_data_id,
        metrics_data_map,
        allowed_value_types={mp.DOUBLE_ARRAY_DATA_TYPE, mp.INDEXED_DOUBLE_ARRAY_DATA_TYPE})

    last_ub = None
    for bucket in histogram_metric_values.buckets:
        if last_ub is None:
            _metrics_assert(bucket.lower == histogram_metric_values.lower_bound) 
        if last_ub is not None:
            _metrics_assert(last_ub == bucket.lower)
        _metrics_assert(bucket.lower < bucket.upper)
        last_ub = bucket.upper
    _metrics_assert(last_ub == histogram_metric_values.upper_bound)
    
  


def _validate_scalar_metric_values(
        scalar_metric_values: mp.ScalarMetricValues, metrics_data_map: dict[str, mp.MetricsData]):
    _metrics_assert(scalar_metric_values.HasField('value'))


def _validate_metric_values(
        metric_values: mp.MetricValues, metrics_data_map: dict[str, mp.MetricsData]):
    _metrics_assert(metric_values.WhichOneof("metric_values") is not None)
    if metric_values.HasField("double_metric_values"):
        _validate_double_metric_values(
            metric_values.double_metric_values,
            metrics_data_map)
    elif metric_values.HasField("double_over_time_metric_values"):
        _validate_double_over_time_metric_values(
            metric_values.double_over_time_metric_values, metrics_data_map)
    elif metric_values.HasField("line_plot_metric_values"):
        _validate_line_plot_metric_values(
            metric_values.line_plot_metric_values, metrics_data_map)
    elif metric_values.HasField("bar_chart_metric_values"):
        _validate_bar_chart_metric_values(
            metric_values.bar_chart_metric_values, metrics_data_map)
    elif metric_values.HasField("states_over_time_metric_values"):
        _validate_states_over_time_metric_values(
            metric_values.states_over_time_metric_values, metrics_data_map)
    elif metric_values.HasField("histogram_metric_values"):
        _validate_histogram_metric_values(
            metric_values.histogram_metric_values, metrics_data_map)
    else: # metric_values.HasField("scalar_metric_values")
        _validate_scalar_metric_values(
            metric_values.scalar_metric_values,
            metrics_data_map)


def _validate_metric(metric: mp.Metric,
                     metrics_data_map: dict[str,
                                            mp.MetricsData]):
    _validate_metric_id(metric.metric_id)
    _metrics_assert(metric.name != "")
    _validate_metric_type(metric.type)
    # No constraints on description
    _validate_metric_status(metric.status)
    # No constraints on should_display
    _validate_metric_values(metric.metric_values, metrics_data_map)
    # No constraints on blocking
    _validate_metric_importance(metric.importance)
    _metrics_assert(metric.HasField("job_id"))
    _validate_job_id(metric.job_id)

    if metric.type == mp.DOUBLE_SUMMARY_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField('double_metric_values'))
    elif metric.type == mp.DOUBLE_OVER_TIME_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField('double_over_time_metric_values'))
    elif metric.type == mp.LINE_PLOT_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField('line_plot_metric_values'))
    elif metric.type == mp.BAR_CHART_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField('bar_chart_metric_values'))
    elif metric.type == mp.STATES_OVER_TIME_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField('states_over_time_metric_values'))
    elif metric.type == mp.HISTOGRAM_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField('histogram_metric_values'))
    else: # metric.type == mp.SCALAR_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField('scalar_metric_values'))


def _validate_job_level_metrics(
        job_level_metrics: mp.MetricCollection, metrics_data_map: dict[str, mp.MetricsData]):
    for metric in job_level_metrics.metrics:
        _validate_metric(metric, metrics_data_map)
    _validate_metric_status(job_level_metrics.metrics_status)


def _validate_array_matches_type(array: mp.Array, data_type: mp.MetricsDataType):
    if data_type in (mp.DOUBLE_ARRAY_DATA_TYPE, mp.INDEXED_DOUBLE_ARRAY_DATA_TYPE):
        _metrics_assert(array.HasField('doubles'))
    elif data_type in (mp.TIMESTAMP_ARRAY_DATA_TYPE, mp.INDEXED_TIMESTAMP_ARRAY_DATA_TYPE):
        _metrics_assert(array.HasField('timestamps'))
    elif data_type in (mp.UUID_ARRAY_DATA_TYPE, mp.INDEXED_UUID_ARRAY_DATA_TYPE):
        _metrics_assert(array.HasField('uuids'))
    elif data_type in (mp.STRING_ARRAY_DATA_TYPE, mp.INDEXED_STRING_ARRAY_DATA_TYPE):
        _metrics_assert(array.HasField('strings'))
    else: # data_type in (mp.METRIC_STATUS_ARRAY_DATA_TYPE, mp.INDEXED_METRIC_STATUS_ARRAY_DATA_TYPE)
        _metrics_assert(array.HasField('statuses'))


def _validate_metrics_data(metrics_data: mp.MetricsData, metrics_data_map: dict[str, mp.MetricsData]):
    _validate_metrics_data_id(metrics_data.metrics_data_id)
    _validate_metrics_data_type(metrics_data.data_type)

    if metrics_data.is_per_category:
        _metrics_assert(metrics_data.HasField('array_per_category'))
        for category, array in metrics_data.array_per_category.category_to_array.items():
            _metrics_assert(_array_length(array) > 0)
            _validate_array_matches_type(array, metrics_data.data_type)
        _metrics_assert(len(metrics_data.category_names) > 0)
        _metrics_assert(set(metrics_data.array_per_category.category_to_array.keys()) == set(metrics_data.category_names))
    else:
        _metrics_assert(metrics_data.HasField('array'))
        _metrics_assert(_array_length(metrics_data.array) > 0)
        _validate_array_matches_type(metrics_data.array, metrics_data.data_type)

    if metrics_data.is_indexed:
        _validate_metrics_data_id(metrics_data.index_data_id)
        id_str = metrics_data.index_data_id.id.data
        _metrics_assert(id_str in metrics_data_map)
        index_data = metrics_data_map[id_str]

        _validate_data_arrays_agree(metrics_data,
                                    index_data)

        _metrics_assert(metrics_data.index_data_type == index_data.data_type)


def _validate_statuses(job_metrics: mp.JobMetrics, metrics_data_map: dict[str, mp.MetricsData]):
    relevant_status_ids = set()
    for metric in job_metrics.job_level_metrics.metrics:
        if not metric.blocking:
            continue
        metric_values = metric.metric_values
        _metrics_assert(metric_values.WhichOneof('metric_values') is not None)
        if metric_values.HasField('double_metric_values'):
            relevant_status_ids.add(metric_values.double_metric_values.status_data_id.id.data)
        elif metric_values.HasField('double_over_time_metric_values'):
            relevant_status_ids.update([data_id.id.data for data_id in metric_values.double_over_time_metric_values.statuses_over_time_data_id])
        elif metric_values.HasField('line_plot_metric_values'):
            relevant_status_ids.update([data_id.id.data for data_id in metric_values.line_plot_metric_values.statuses_data_id])
        elif metric_values.HasField('bar_chart_metric_values'):
            relevant_status_ids.update([data_id.id.data for data_id in metric_values.bar_chart_metric_values.statuses_data_id])
        elif metric_values.HasField('states_over_time_metric_values'):
            relevant_status_ids.update([data_id.id.data for data_id in metric_values.states_over_time_metric_values.statuses_over_time_data_id])
        elif metric_values.HasField('histogram_metric_values'):
            relevant_status_ids.add(metric_values.histogram_metric_values.statuses_data_id.id.data)

    expected_status = mp.PASSED_METRIC_STATUS
    for status_id in relevant_status_ids:
        status_data = metrics_data_map[status_id]

        if status_data.HasField('array'):
            _metrics_assert(status_data.array.HasField('statuses'))
            for status in status_data.array.statuses.array:
                if status == mp.FAILED_METRIC_STATUS:
                    expected_status = mp.FAILED_METRIC_STATUS
        else:
            for key, array in status_data.array_per_category.category_to_array.items():
                _metrics_assert(array.HasField('statuses'))
                for status in array.statuses.array:
                    if status == mp.FAILED_METRIC_STATUS:
                        expected_status = mp.FAILED_METRIC_STATUS
    _metrics_assert(expected_status == job_metrics.metrics_status)
    _metrics_assert(job_metrics.job_level_metrics.metrics_status == job_metrics.metrics_status)

def build_metrics_data_map(
        job_metrics: mp.JobMetrics) -> dict[str, mp.MetricsData]:
    result = {}
    for data in job_metrics.metrics_data:
        _validate_metrics_data_id(data.metrics_data_id)
        result[data.metrics_data_id.id.data] = data

    # Checks uniqueness of ids
    _metrics_assert(len(result) == len(job_metrics.metrics_data))
    return result


def validate_job_metrics(job_metrics: mp.JobMetrics):
    metrics_data_map = build_metrics_data_map(job_metrics)

    _validate_job_id(job_metrics.job_id)
    _validate_job_level_metrics(
        job_metrics.job_level_metrics,
        metrics_data_map)
    _validate_metric_status(job_metrics.metrics_status)

    for metric in job_metrics.job_level_metrics.metrics:
        _metrics_assert(metric.job_id == job_metrics.job_id)

    for metric_data in job_metrics.metrics_data:
        _validate_metrics_data(metric_data, metrics_data_map)

    _validate_statuses(job_metrics, metrics_data_map)
