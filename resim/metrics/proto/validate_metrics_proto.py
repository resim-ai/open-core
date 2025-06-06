# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
validate_metrics_proto.py

This module defines the public function validate_job_metrics() which is used to
validate that a resim.metrics.proto.JobMetrics protobuf message contains valid
contents that can be posted to the metrics endpoint.
"""

import re
import uuid
import math

import google.protobuf.timestamp_pb2 as timestamp_proto
import resim.utils.proto.uuid_pb2 as uuid_proto

import resim.metrics.proto.metrics_pb2 as mp


class InvalidMetricsException(Exception):
    """A simple exception to raise when our metrics are invalid"""


def _metrics_assert(val: bool, msg: str = "") -> None:
    """A function to assert conditions on metrics"""
    if not val:
        raise InvalidMetricsException(msg)


def _validate_hex_color(hex_color: str) -> None:
    # Hex color with 3 or 4 bytes
    _metrics_assert(re.search(r"^#([0-9a-fA-F]{2}){3,4}$", hex_color) is not None)


def _validate_metrics_data_type(metrics_data_type: mp.MetricsDataType) -> None:
    _metrics_assert(metrics_data_type != mp.NO_DATA_TYPE)


def _validate_metric_status(metric_status: mp.MetricStatus) -> None:
    _metrics_assert(metric_status != mp.NO_METRIC_STATUS)


def _validate_metric_type(metric_type: mp.MetricType) -> None:
    _metrics_assert(metric_type != mp.NO_METRIC_TYPE)


def _validate_metric_importance(importance: mp.MetricImportance) -> None:
    _metrics_assert(importance != mp.NO_SPECIFIED_IMPORTANCE)


def _validate_timestamp_type(timestamp_type: mp.TimestampType) -> None:
    _metrics_assert(timestamp_type != mp.NO_TYPE)


def _validate_uuid(unique_id: uuid_proto.UUID) -> None:
    """Check that the given UUID proto is a valid UUID"""
    try:
        uuid.UUID(hex=unique_id.data)
    except ValueError as exc:
        raise InvalidMetricsException("Invalid UUID.") from exc


def _validate_metrics_data_id(metrics_data_id: mp.MetricsDataId) -> None:
    _validate_uuid(metrics_data_id.id)


def _validate_job_id(job_id: mp.JobId) -> None:
    _validate_uuid(job_id.id)


def _validate_tags(tags: mp.Tag) -> None:
    for tag in tags:
        if len(tag.key) == 0 or len(tag.value) == 0:
            raise InvalidMetricsException()


def _validate_metric_id(metric_id: mp.MetricId) -> None:
    _validate_uuid(metric_id.id)


def _validate_event_id(event_id: mp.EventId) -> None:
    _validate_uuid(event_id.id)


def _validate_timestamp(timestamp: timestamp_proto.Timestamp) -> None:
    _metrics_assert(timestamp.seconds >= 0)
    _metrics_assert(timestamp.nanos >= 0)
    _metrics_assert(timestamp.nanos < 1e9)


_ALL_SERIES_DATA_TYPES = {
    mp.DOUBLE_SERIES_DATA_TYPE,
    mp.TIMESTAMP_SERIES_DATA_TYPE,
    mp.UUID_SERIES_DATA_TYPE,
    mp.STRING_SERIES_DATA_TYPE,
    mp.METRIC_STATUS_SERIES_DATA_TYPE,
    mp.INDEXED_DOUBLE_SERIES_DATA_TYPE,
    mp.INDEXED_TIMESTAMP_SERIES_DATA_TYPE,
    mp.INDEXED_UUID_SERIES_DATA_TYPE,
    mp.INDEXED_STRING_SERIES_DATA_TYPE,
    mp.INDEXED_METRIC_STATUS_SERIES_DATA_TYPE,
}


def is_indexed(data_type: mp.MetricsDataType) -> bool:
    """Is this one of the indexed series data types or not?"""
    return data_type in (
        mp.INDEXED_DOUBLE_SERIES_DATA_TYPE,
        mp.INDEXED_TIMESTAMP_SERIES_DATA_TYPE,
        mp.INDEXED_UUID_SERIES_DATA_TYPE,
        mp.INDEXED_STRING_SERIES_DATA_TYPE,
        mp.INDEXED_METRIC_STATUS_SERIES_DATA_TYPE,
    )


def _series_length(series: mp.Series) -> int:
    """Get the length of an series regardless of its type"""
    _metrics_assert(series.WhichOneof("series") is not None)
    if series.HasField("doubles"):
        return len(series.doubles.series)
    if series.HasField("timestamps"):
        return len(series.timestamps.series)
    if series.HasField("uuids"):
        return len(series.uuids.series)
    if series.HasField("strings"):
        return len(series.strings.series)
    # if series.HasField("statuses"):
    # This is commented out for code coverage reasons
    return len(series.statuses.series)


def _validate_data_series_agree(data_a: mp.MetricsData, data_b: mp.MetricsData) -> None:
    """
    Validate that the data schema for two MetricsDatas agree

    When we say "agree", we mean that the contained data are the same type
    (series or series_per_category)
    *and*

      - if that type is "series" they have the same length

      - if that type is "series_per_category"  they have the same categories and
        each category has the same length in each series.

    Args:
        data_a: The first MetricsData to compare.
        data_b: The second MetricsData to compare.

    Raises:
        InvalidMetricsException if the data schemas don't match.
    """

    _metrics_assert(data_a.WhichOneof("data") is not None)
    _metrics_assert(data_a.WhichOneof("data") == data_b.WhichOneof("data"))
    if data_a.HasField("series"):
        _metrics_assert(_series_length(data_a.series) == _series_length(data_b.series))
    else:  # data_a.HasField("series_per_category")
        data_a_series = data_a.series_per_category.category_to_series
        data_b_series = data_b.series_per_category.category_to_series

        _metrics_assert(len(data_a_series) == len(data_b_series))
        for key in data_a_series:
            _metrics_assert(key in data_b_series)
            _metrics_assert(
                _series_length(data_a_series[key]) == _series_length(data_b_series[key])
            )


def _validate_values_and_statuses(
    value_data_id: mp.MetricsDataId,
    status_data_id: mp.MetricsDataId,
    metrics_data_map: dict[str, mp.MetricsData],
    *,
    allowed_value_types: set[mp.MetricsDataType],
    allowed_index_types: set[mp.MetricsDataType],
) -> None:
    """
    Validate that the given value data and status data match.

    Here we check that the status data is indexed if and only if the value data
    is indexed, and confirm that both the value data and status data use the
    same index data if they are indexed. We also validate that the status data
    has INDEX_METRIC_STATUS_SERIES_DATA_TYPE or METRIC_STATUS_SERIES_DATA_TYPE as
    appropriate.

    Furthermore, we also allow users to specify restrictions on the allowed
    value types and the allowed index data types using optional arguments.

    Args:
        value_data_id: The id of the value data we want to check.
        status_data_id: The id of the status data we want to check.
        metrics_data_map: A map of metrics data ids to metrics data objects
                          where we can find the value and status data.
        allowed_value_types: The data types allowed for the value data.
        allowed_index_types: The data types allowed for the indexed data.
    """
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
        _metrics_assert(status_data.data_type == mp.METRIC_STATUS_SERIES_DATA_TYPE)
    else:
        _metrics_assert(
            status_data.data_type == mp.INDEXED_METRIC_STATUS_SERIES_DATA_TYPE
        )
        _metrics_assert(status_data.index_data_id == value_data.index_data_id)
        _metrics_assert(status_data.index_data_type in allowed_index_types)
        _metrics_assert(value_data.index_data_type in allowed_index_types)

    _validate_data_series_agree(value_data, status_data)


def _validate_double_metric_values(
    double_metric_values: mp.DoubleSummaryMetricValues,
    metrics_data_map: dict[str, mp.MetricsData],
) -> None:
    """
    Check that a DoubleSummaryMetricValues is valid.

    Args:
        double_metric_values: The metric values to check.
        metrics_data_map: A map to find the metrics data in.
    """

    if double_metric_values.HasField("timestamp_index"):
        allowed_value_types = {mp.INDEXED_DOUBLE_SERIES_DATA_TYPE}
        allowed_index_types = {
            mp.TIMESTAMP_SERIES_DATA_TYPE,
            mp.INDEXED_TIMESTAMP_SERIES_DATA_TYPE,
        }
    elif double_metric_values.HasField("uuid_index"):
        allowed_value_types = {mp.INDEXED_DOUBLE_SERIES_DATA_TYPE}
        allowed_index_types = {
            mp.UUID_SERIES_DATA_TYPE,
            mp.INDEXED_UUID_SERIES_DATA_TYPE,
        }
    elif double_metric_values.HasField("string_index"):
        allowed_value_types = {mp.INDEXED_DOUBLE_SERIES_DATA_TYPE}
        allowed_index_types = {
            mp.STRING_SERIES_DATA_TYPE,
            mp.INDEXED_STRING_SERIES_DATA_TYPE,
        }
    else:
        # series_index or no index
        allowed_value_types = {
            mp.DOUBLE_SERIES_DATA_TYPE,
            mp.INDEXED_DOUBLE_SERIES_DATA_TYPE,
        }
        allowed_index_types = _ALL_SERIES_DATA_TYPES

    _validate_values_and_statuses(
        double_metric_values.value_data_id,
        double_metric_values.status_data_id,
        metrics_data_map,
        allowed_value_types=allowed_value_types,
        allowed_index_types=allowed_index_types,
    )

    # No constraints on failure_definition


def _validate_double_over_time_metric_values(
    double_over_time_metric_values: mp.DoubleOverTimeMetricValues,
    metrics_data_map: dict[str, mp.MetricsData],
) -> None:
    """
    Check that a DoubleOverTimeMetricValues is valid.

    Args:
        double_over_time_metric_values: The metric values to check.
        metrics_data_map: A map to find the metrics data in.
    """
    length = len(double_over_time_metric_values.doubles_over_time_data_id)
    _metrics_assert(
        length == len(double_over_time_metric_values.statuses_over_time_data_id)
    )
    _metrics_assert(length == len(double_over_time_metric_values.failure_definition))

    for i in range(length):
        _validate_values_and_statuses(
            double_over_time_metric_values.doubles_over_time_data_id[i],
            double_over_time_metric_values.statuses_over_time_data_id[i],
            metrics_data_map,
            allowed_value_types={mp.INDEXED_DOUBLE_SERIES_DATA_TYPE},
            allowed_index_types={
                mp.TIMESTAMP_SERIES_DATA_TYPE,
                mp.INDEXED_TIMESTAMP_SERIES_DATA_TYPE,
            },
        )


def _validate_line_plot_metric_values(
    line_plot_metric_values: mp.LinePlotMetricValues,
    metrics_data_map: dict[str, mp.MetricsData],
) -> None:
    """
    Check that a LinePlotMetricValues is valid.

    Args:
        line_plot_metric_values: The metric values to check.
        metrics_data_map: A map to find the metrics data in.
    """
    length = len(line_plot_metric_values.x_doubles_data_id)
    _metrics_assert(length == len(line_plot_metric_values.y_doubles_data_id))
    _metrics_assert(length == len(line_plot_metric_values.statuses_data_id))

    for i in range(length):
        statuses_data_id = line_plot_metric_values.statuses_data_id[i]
        for values_data_id in (
            line_plot_metric_values.x_doubles_data_id[i],
            line_plot_metric_values.y_doubles_data_id[i],
        ):
            _validate_values_and_statuses(
                values_data_id,
                statuses_data_id,
                metrics_data_map,
                allowed_value_types={
                    mp.DOUBLE_SERIES_DATA_TYPE,
                    mp.INDEXED_DOUBLE_SERIES_DATA_TYPE,
                },
                allowed_index_types={
                    mp.TIMESTAMP_SERIES_DATA_TYPE,
                    mp.INDEXED_TIMESTAMP_SERIES_DATA_TYPE,
                },
            )


def _validate_bar_chart_metric_values(
    bar_chart_metric_values: mp.BarChartMetricValues,
    metrics_data_map: dict[str, mp.MetricsData],
) -> None:
    """
    Check that a BarChartMetricValues is valid.

    Args:
        bar_chart_metric_values: The metric values to check.
        metrics_data_map: A map to find the metrics data in.
    """
    length = len(bar_chart_metric_values.values_data_id)
    _metrics_assert(length == len(bar_chart_metric_values.statuses_data_id))

    for i in range(length):
        _validate_values_and_statuses(
            bar_chart_metric_values.values_data_id[i],
            bar_chart_metric_values.statuses_data_id[i],
            metrics_data_map,
            allowed_value_types={mp.INDEXED_DOUBLE_SERIES_DATA_TYPE},
            allowed_index_types={
                mp.STRING_SERIES_DATA_TYPE,
                mp.INDEXED_STRING_SERIES_DATA_TYPE,
            },
        )


def _validate_batchwise_bar_chart_metric_values(
    batchwise_bar_chart_metric_values: mp.BatchwiseBarChartMetricValues,
    metrics_data_map: dict[str, mp.MetricsData],
) -> None:
    """
    Check that a BatchwiseBarChartMetricValues is valid.

    Args:
        batchwise_bar_chart_metric_values: The metric values to check.
        metrics_data_map: A map to find the metrics data in.
    """
    values = batchwise_bar_chart_metric_values
    length = len(values.values_data_id)
    _metrics_assert(length == len(values.times_data_id))
    _metrics_assert(length == len(values.statuses_data_id))
    _metrics_assert(length == len(values.categories))

    for color in values.colors:
        _validate_hex_color(color)

    _validate_uuid(values.project_id)

    for i in range(length):
        _validate_values_and_statuses(
            values.values_data_id[i],
            values.statuses_data_id[i],
            metrics_data_map,
            allowed_value_types={mp.INDEXED_DOUBLE_SERIES_DATA_TYPE},
            allowed_index_types={
                mp.UUID_SERIES_DATA_TYPE,
                mp.INDEXED_UUID_SERIES_DATA_TYPE,
            },
        )
        _validate_values_and_statuses(
            values.times_data_id[i],
            values.statuses_data_id[i],
            metrics_data_map,
            allowed_value_types={mp.INDEXED_TIMESTAMP_SERIES_DATA_TYPE},
            allowed_index_types={
                mp.UUID_SERIES_DATA_TYPE,
                mp.INDEXED_UUID_SERIES_DATA_TYPE,
            },
        )


def _validate_states_over_time_metric_values(
    states_over_time_metric_values: mp.StatesOverTimeMetricValues,
    metrics_data_map: dict[str, mp.MetricsData],
) -> None:
    """
    Check that a StatesOverTimeMetricValues is valid.

    Args:
        states_over_time_metric_values: The metric values to check.
        metrics_data_map: A map to find the metrics data in.
    """
    length = len(states_over_time_metric_values.states_over_time_data_id)
    _metrics_assert(
        length == len(states_over_time_metric_values.statuses_over_time_data_id)
    )

    for i in range(length):
        value_data_id = states_over_time_metric_values.states_over_time_data_id[i]
        _validate_values_and_statuses(
            value_data_id,
            states_over_time_metric_values.statuses_over_time_data_id[i],
            metrics_data_map,
            allowed_value_types={mp.INDEXED_STRING_SERIES_DATA_TYPE},
            allowed_index_types={
                mp.TIMESTAMP_SERIES_DATA_TYPE,
                mp.INDEXED_TIMESTAMP_SERIES_DATA_TYPE,
            },
        )

        # Check that all the states are in the states set
        id_str = value_data_id.id.data
        value_data = metrics_data_map[id_str]
        if value_data.HasField("series"):
            _metrics_assert(value_data.series.HasField("strings"))
            for state in value_data.series.strings.series:
                _metrics_assert(state in states_over_time_metric_values.states_set)
        else:
            for _, series in value_data.series_per_category.category_to_series.items():
                _metrics_assert(series.HasField("strings"))
                for state in series.strings.series:
                    _metrics_assert(state in states_over_time_metric_values.states_set)

        _metrics_assert(
            set(states_over_time_metric_values.failure_states).issubset(
                states_over_time_metric_values.states_set
            )
        )


def _validate_histogram_metric_values(
    histogram_metric_values: mp.HistogramMetricValues,
    metrics_data_map: dict[str, mp.MetricsData],
) -> None:
    """
    Check that a HistogramMetricValues is valid.

    Args:
        histogram_metric_values: The metric values to check.
        metrics_data_map: A map to find the metrics data in.
    """
    _validate_values_and_statuses(
        histogram_metric_values.values_data_id,
        histogram_metric_values.statuses_data_id,
        metrics_data_map,
        allowed_value_types={
            mp.DOUBLE_SERIES_DATA_TYPE,
            mp.INDEXED_DOUBLE_SERIES_DATA_TYPE,
        },
        allowed_index_types=_ALL_SERIES_DATA_TYPES,
    )

    last_ub = None
    for bucket in histogram_metric_values.buckets:
        if last_ub is None:
            _metrics_assert(bucket.lower == histogram_metric_values.lower_bound)
        if last_ub is not None:
            _metrics_assert(last_ub == bucket.lower)
        _metrics_assert(bucket.lower < bucket.upper)
        last_ub = bucket.upper
    _metrics_assert(last_ub == histogram_metric_values.upper_bound)


def _validate_scalar_metric_values(scalar_metric_values: mp.ScalarMetricValues) -> None:
    """
    Check that a ScalarMetricValues is valid.

    Args:
        scalar_metric_values: The metric values to check.
    """
    _metrics_assert(scalar_metric_values.HasField("value"))
    _metrics_assert(
        not math.isnan(scalar_metric_values.value), "Scalar metric value cannot be NaN"
    )


def _validate_plotly_metric_values(plotly_metric_values: mp.PlotlyMetricValues) -> None:
    """
    Check that a PlotlyMetricValues is valid.

    Args:
        plotly_metric_values: The metric values to check.
    """
    _metrics_assert(plotly_metric_values.HasField("json"))


def _validate_text_metric_values(text_metric_values: mp.TextMetricValues) -> None:
    """
    Check that a TextMetricValues is valid.

    Args:
        text_metric_values: The metric values to check.
    """
    _metrics_assert(text_metric_values.text != "")


def _validate_image_metric_values(
    image_metric_values: mp.ImageMetricValues,
    metrics_data_map: dict[str, mp.MetricsData],
) -> None:
    """
    Check that an ImageMetricValues is valid.

    Args:
        image_metric_values: The metric values to check.
        metrics_data_map: A map to find the metrics data in.
    """
    _metrics_assert(image_metric_values.HasField("image_data_id"))
    image_data_id = image_metric_values.image_data_id
    _validate_metrics_data_id(image_data_id)
    id_str = image_data_id.id.data
    _metrics_assert(id_str in metrics_data_map)
    value_data = metrics_data_map[id_str]
    _metrics_assert(value_data.data_type == mp.EXTERNAL_FILE_DATA_TYPE)


def _validate_image_list_metric_values(
    image_list_metric_values: mp.ImageListMetricValues,
    metrics_data_map: dict[str, mp.MetricsData],
) -> None:
    """
    Check that an ImageListMetricValues is valid.

    Args:
        image_list_metric_values: The metric values to check.
        metrics_data_map: A map to find the metrics data in.
    """
    _metrics_assert(len(image_list_metric_values.image_data_ids) > 0)
    image_data_ids = image_list_metric_values.image_data_ids
    for image_data_id in image_data_ids:
        _validate_metrics_data_id(image_data_id)
        id_str = image_data_id.id.data
        _metrics_assert(id_str in metrics_data_map)
        value_data = metrics_data_map[id_str]
        _metrics_assert(value_data.data_type == mp.EXTERNAL_FILE_DATA_TYPE)


def _validate_metric_values(
    metric_values: mp.MetricValues, metrics_data_map: dict[str, mp.MetricsData]
) -> None:
    """
    Check that a MetricValues is valid.

    Args:
        metric_values: The metric values to check.
        metrics_data_map: A map to find the metrics data in.
    """
    _metrics_assert(metric_values.WhichOneof("metric_values") is not None)
    if metric_values.HasField("double_metric_values"):
        _validate_double_metric_values(
            metric_values.double_metric_values, metrics_data_map
        )
    elif metric_values.HasField("double_over_time_metric_values"):
        _validate_double_over_time_metric_values(
            metric_values.double_over_time_metric_values, metrics_data_map
        )
    elif metric_values.HasField("line_plot_metric_values"):
        _validate_line_plot_metric_values(
            metric_values.line_plot_metric_values, metrics_data_map
        )
    elif metric_values.HasField("bar_chart_metric_values"):
        _validate_bar_chart_metric_values(
            metric_values.bar_chart_metric_values, metrics_data_map
        )
    elif metric_values.HasField("batchwise_bar_chart_metric_values"):
        _validate_batchwise_bar_chart_metric_values(
            metric_values.batchwise_bar_chart_metric_values, metrics_data_map
        )
    elif metric_values.HasField("states_over_time_metric_values"):
        _validate_states_over_time_metric_values(
            metric_values.states_over_time_metric_values, metrics_data_map
        )
    elif metric_values.HasField("histogram_metric_values"):
        _validate_histogram_metric_values(
            metric_values.histogram_metric_values, metrics_data_map
        )
    elif metric_values.HasField("scalar_metric_values"):
        _validate_scalar_metric_values(metric_values.scalar_metric_values)
    elif metric_values.HasField("plotly_metric_values"):
        _validate_plotly_metric_values(metric_values.plotly_metric_values)
    elif metric_values.HasField("text_metric_values"):
        _validate_text_metric_values(metric_values.text_metric_values)
    elif metric_values.HasField("image_list_metric_values"):
        _validate_image_list_metric_values(
            metric_values.image_list_metric_values, metrics_data_map
        )
    else:  # metric_values.HasField("image_metric_values")
        _validate_image_metric_values(
            metric_values.image_metric_values, metrics_data_map
        )


def _validate_metric_used_in_event(
    metric_id: mp.MetricId, events_list: list[mp.Event]
) -> bool:
    """
    Check that the metric_id is valid and used in an event.

    Args:
        metric_id: The metric_id to check.
        events_list: A list of the events.
    """
    for event in events_list:
        if metric_id in event.metrics:
            return True
    return False


def _validate_metric(
    metric: mp.Metric,
    metrics_data_map: dict[str, mp.MetricsData],
    events_list: list[mp.Event],
) -> None:
    """
    Check that a Metric is valid.

    Args:
        metric: The metric to check.
        metrics_data_map: A map to find the metrics data in.
        events_list: A list of the events.
    """
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
    _validate_tags(metric.tags)

    if metric.event_metric:
        _validate_metric_used_in_event(metric.metric_id, events_list)

    if metric.type == mp.DOUBLE_SUMMARY_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField("double_metric_values"))
    elif metric.type == mp.DOUBLE_OVER_TIME_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField("double_over_time_metric_values"))
    elif metric.type == mp.LINE_PLOT_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField("line_plot_metric_values"))
    elif metric.type == mp.BAR_CHART_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField("bar_chart_metric_values"))
    elif metric.type == mp.BATCHWISE_BAR_CHART_METRIC_TYPE:
        _metrics_assert(
            metric.metric_values.HasField("batchwise_bar_chart_metric_values")
        )
    elif metric.type == mp.STATES_OVER_TIME_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField("states_over_time_metric_values"))
    elif metric.type == mp.HISTOGRAM_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField("histogram_metric_values"))
    elif metric.type == mp.SCALAR_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField("scalar_metric_values"))
    elif metric.type == mp.PLOTLY_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField("plotly_metric_values"))
    elif metric.type == mp.TEXT_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField("text_metric_values"))
    elif metric.type == mp.IMAGE_LIST_METRIC_TYPE:
        _metrics_assert(metric.metric_values.HasField("image_list_metric_values"))
    else:  # mp.IMAGE_METRIC_TYPE
        _metrics_assert(metric.metric_values.HasField("image_metric_values"))


def _validate_job_level_metrics(
    job_level_metrics: mp.MetricCollection,
    metrics_data_map: dict[str, mp.MetricsData],
    events_list: list[mp.Event],
) -> None:
    """
    Check that a MetricCollection is valid.

    Args:
        job_level_metrics: The MetricCollection to check.
        metrics_data_map: A map to find the metrics data in.
        events_list: A list of all the events
    """
    for metric in job_level_metrics.metrics:
        _validate_metric(metric, metrics_data_map, events_list)
    _validate_metric_status(job_level_metrics.metrics_status)


def _validate_series_matches_type(
    series: mp.Series, data_type: mp.MetricsDataType
) -> None:
    """
    Check that a metric data series matches a given type.

    Args:
        series: The series to check
        data_type: The type we expect it to contain
    """
    if data_type in (mp.DOUBLE_SERIES_DATA_TYPE, mp.INDEXED_DOUBLE_SERIES_DATA_TYPE):
        _metrics_assert(series.HasField("doubles"))
    elif data_type in (
        mp.TIMESTAMP_SERIES_DATA_TYPE,
        mp.INDEXED_TIMESTAMP_SERIES_DATA_TYPE,
    ):
        _metrics_assert(series.HasField("timestamps"))
    elif data_type in (mp.UUID_SERIES_DATA_TYPE, mp.INDEXED_UUID_SERIES_DATA_TYPE):
        _metrics_assert(series.HasField("uuids"))
    elif data_type in (mp.STRING_SERIES_DATA_TYPE, mp.INDEXED_STRING_SERIES_DATA_TYPE):
        _metrics_assert(series.HasField("strings"))
    # data_type in (mp.METRIC_STATUS_SERIES_DATA_TYPE,
    # mp.INDEXED_METRIC_STATUS_SERIES_DATA_TYPE):
    else:
        _metrics_assert(series.HasField("statuses"))


def _validate_metrics_data(
    metrics_data: mp.MetricsData, metrics_data_map: dict[str, mp.MetricsData]
) -> None:
    """
    Check that the MetricsData is valid.

    Args:
        metrics_data: The MetricsData to validate.
        metrics_data_map: A map to find the metrics data in.
    """
    _validate_metrics_data_id(metrics_data.metrics_data_id)
    _validate_metrics_data_type(metrics_data.data_type)

    if metrics_data.data_type == mp.EXTERNAL_FILE_DATA_TYPE:
        _metrics_assert(metrics_data.HasField("external_file"))
        _metrics_assert(len(metrics_data.external_file.path) > 0)
    elif metrics_data.is_per_category:
        _metrics_assert(metrics_data.HasField("series_per_category"))
        for _, series in metrics_data.series_per_category.category_to_series.items():
            _metrics_assert(_series_length(series) > 0)
            _validate_series_matches_type(series, metrics_data.data_type)
        _metrics_assert(len(metrics_data.category_names) > 0)
        _metrics_assert(
            set(metrics_data.series_per_category.category_to_series.keys())
            == set(metrics_data.category_names)
        )
    else:
        _metrics_assert(metrics_data.HasField("series"))
        _metrics_assert(_series_length(metrics_data.series) > 0)
        _validate_series_matches_type(metrics_data.series, metrics_data.data_type)

    if metrics_data.is_indexed:
        _validate_metrics_data_id(metrics_data.index_data_id)
        id_str = metrics_data.index_data_id.id.data
        _metrics_assert(id_str in metrics_data_map)
        index_data = metrics_data_map[id_str]

        _validate_data_series_agree(metrics_data, index_data)

        _metrics_assert(metrics_data.index_data_type == index_data.data_type)


def _validate_event(event: mp.Event, metrics_map: dict[str, mp.Metric]) -> None:
    """
    Check that the Event is valid.

    Args:
        event: The Event to validate.
        metrics_map: A map to find the metric in.
    """
    _validate_event_id(event.event_id)
    _metrics_assert(event.name != "", "Event name cannot be empty.")
    # No constraints on description
    # No constraints on tags

    # Validate that each metric id is in the metrics_map
    names = set()
    for metric_id in event.metrics:
        _validate_metric_id(metric_id)
        _metrics_assert(
            metric_id.id.data in metrics_map,
            f"Event metric id {metric_id.id.data} not found in metrics map.",
        )
        # Check name is unique within the event
        metric = metrics_map[metric_id.id.data]
        _metrics_assert(
            metric.name not in names,
            f"Event metric names must be unique within the event: {event.name}.",
        )
        names.add(metric.name)

    _validate_metric_status(event.status)

    # Validate that the type is either absolute or relative
    _validate_timestamp_type(event.timestamp_type)
    _validate_timestamp(event.timestamp)

    _validate_metric_importance(event.importance)


def build_metrics_data_map(job_metrics: mp.JobMetrics) -> dict[str, mp.MetricsData]:
    """
    Build a map of id (as a str) to mp.MetricsData.

    We do this so we can quickly look these up while validating.

    Args:
        job_metrics: The job metrics to get the data from.

    Returns:
        A map of id -> mp.MetricsData from the job_level_metrics.metrics field
        in the job_metrics.
    """
    result = {}
    for data in job_metrics.metrics_data:
        _validate_metrics_data_id(data.metrics_data_id)
        result[data.metrics_data_id.id.data] = data

    # Checks uniqueness of ids
    _metrics_assert(len(result) == len(job_metrics.metrics_data))
    return result


def build_metrics_map(job_metrics: mp.JobMetrics) -> dict[str, mp.Metric]:
    """
    Build a map of id (as a str) to mp.Metric.

    We do this so we can quickly look these up while validating.

    Args:
        job_metrics: The job metrics to get the data from.

    Returns:
        A map of id -> mp.Metric from the job_level_metrics.metrics field
        in the job_metrics.
    """
    result = {}
    for metric in job_metrics.job_level_metrics.metrics:
        _validate_metric_id(metric.metric_id)
        result[metric.metric_id.id.data] = metric

    # Checks uniqueness of ids
    _metrics_assert(len(result) == len(job_metrics.job_level_metrics.metrics))
    return result


def build_events_list(job_metrics: mp.JobMetrics) -> list[mp.Event]:
    """
    Build a list of mp.Event.

    We do this so we can quickly look these up while validating.

    Args:
        job_metrics: The job metrics to get the data from.

    Returns:
        A list mp.Event from the events field
        in the job_metrics.
    """
    result = []
    for event in job_metrics.events:
        _validate_event_id(event.event_id)
        result.append(event)

    # Checks uniqueness of ids
    _metrics_assert(len(result) == len(job_metrics.events))
    return result


def validate_job_metrics(job_metrics: mp.JobMetrics) -> None:
    """
    Validate that the given JobMetrics is valid and can be posted to the
    endpoint.
    """
    metrics_data_map = build_metrics_data_map(job_metrics)
    metrics_map = build_metrics_map(job_metrics)
    events_list = build_events_list(job_metrics)

    _validate_job_id(job_metrics.job_id)
    _validate_job_level_metrics(
        job_metrics.job_level_metrics, metrics_data_map, events_list
    )
    _validate_metric_status(job_metrics.metrics_status)

    # Use a set to check for duplicated names for non-event metrics
    metric_names = set()
    for metric in job_metrics.job_level_metrics.metrics:
        if not metric.event_metric:
            _metrics_assert(
                metric.name not in metric_names,
                f"Metric name {metric.name} is not unique.",
            )
            metric_names.add(metric.name)
        _metrics_assert(
            metric.job_id == job_metrics.job_id,
            "Metric job ID must match the writer's job ID.",
        )

    # Use a set to check for duplicated names
    metric_data_names = set()
    for metric_data in job_metrics.metrics_data:
        _metrics_assert(metric_data.name not in metric_data_names)
        metric_data_names.add(metric_data.name)
        _validate_metrics_data(metric_data, metrics_data_map)

    if job_metrics.events:
        # Use a set to check for duplicated names
        event_names = set()

        # Validate that all events per-job use the same timestamp type:
        timestamp_type = job_metrics.events[0].timestamp_type
        for event in job_metrics.events:
            _metrics_assert(event.name not in event_names)
            _metrics_assert(event.timestamp_type == timestamp_type)
            event_names.add(event.name)
            _validate_event(event, metrics_map)
