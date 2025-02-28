# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""This module contains a free function used to unpack a set of
metrics and metrics data protos corresponding to a single job into a
set of Metric and MetricsData objects referencing each other as
appropriate.
"""

import uuid
from dataclasses import dataclass
from typing import Any, Callable, Optional, cast

import google.protobuf.json_format as pjf
import numpy as np
import resim.utils.proto.uuid_pb2 as uuid_proto

import resim.metrics.proto.metrics_pb2 as mp
from resim.metrics.python.metrics import (
    BarChartMetric,
    BaseMetricsData,
    BatchwiseBarChartMetric,
    DoubleOverTimeMetric,
    DoubleSummaryMetric,
    Event,
    ExternalFileMetricsData,
    GroupedMetricsData,
    HistogramMetric,
    ImageListMetric,
    ImageMetric,
    LinePlotMetric,
    Metric,
    MetricsData,
    PlotlyMetric,
    ScalarMetric,
    SeriesMetricsData,
    StatesOverTimeMetric,
    TextMetric,
)
from resim.metrics.python.metrics_utils import (
    DoubleFailureDefinition,
    HistogramBucket,
    MetricImportance,
    MetricStatus,
    Timestamp,
)


@dataclass
class UnpackedMetrics:
    """A class representing unpacked metrics."""

    metrics: list[Metric]
    metrics_data: list[BaseMetricsData]
    events: list[Event]
    names: set[str]


def unpack_metrics(
    *,
    metrics: list[mp.Metric],
    metrics_data: list[mp.MetricsData],
    events: list[mp.Event],
) -> UnpackedMetrics:
    """The main unpacker for metrics, metrics data, and events

    This function reads a list of metrics, metrics data, and event protobuf messages and
    unpacks them into Metric, MetricsData, and Event classes wrapped in the
    UnpackedMetrics class.
    """

    id_to_metrics_data_map = {
        _unpack_uuid(md.metrics_data_id.id): md for md in metrics_data
    }

    id_to_unpacked_metrics_data: dict[uuid.UUID, BaseMetricsData] = {}

    id_to_unpacked_metrics: dict[uuid.UUID, Metric] = {}

    def recursive_unpack_metrics_data(current_id: uuid.UUID) -> None:
        if current_id in id_to_unpacked_metrics_data:
            return
        metrics_data = id_to_metrics_data_map[current_id]

        if metrics_data.is_indexed:
            index_id = _unpack_uuid(metrics_data.index_data_id.id)
            recursive_unpack_metrics_data(index_id)
            assert index_id in id_to_unpacked_metrics_data

        _unpack_metrics_data(metrics_data, id_to_unpacked_metrics_data)

    for metrics_data_id in id_to_metrics_data_map:
        recursive_unpack_metrics_data(metrics_data_id)

    unpacked_metrics = []
    for metric in metrics:
        unpacked_metrics.append(
            _unpack_metric(metric, id_to_unpacked_metrics_data, id_to_unpacked_metrics)
        )

    unpacked_metrics_data = list(id_to_unpacked_metrics_data.values())

    unpacked_events = []
    for event in events:
        unpacked_events.append(_unpack_event(event, id_to_unpacked_metrics))

    names = (
        {metric.name for metric in unpacked_metrics}
        | {metrics_data.name for metrics_data in unpacked_metrics_data}
        | {events.name for events in unpacked_events}
    )

    # Enforce name uniqueness
    assert len(names) == len(unpacked_metrics) + len(unpacked_metrics_data) + len(
        unpacked_events
    )

    return UnpackedMetrics(
        metrics=unpacked_metrics,
        metrics_data=unpacked_metrics_data,
        events=unpacked_events,
        names=names,
    )


def _unpack_uuid(uuid_msg: uuid_proto.UUID) -> uuid.UUID:
    return uuid.UUID(uuid_msg.data)


def _unpack_series(series_proto: mp.Series) -> np.ndarray:
    data_case = series_proto.WhichOneof("series")
    if data_case == "doubles":
        return np.array(series_proto.doubles.series, dtype=np.float64)
    if data_case == "timestamps":
        return np.array(
            [Timestamp.unpack(ts) for ts in series_proto.timestamps.series],
            dtype=Timestamp,
        )
    if data_case == "uuids":
        return np.array(
            [_unpack_uuid(element) for element in series_proto.uuids.series],
            dtype=uuid.UUID,
        )
    if data_case == "strings":
        return np.array(series_proto.strings.series, dtype=str)
    assert data_case == "statuses"
    return np.array(
        [MetricStatus(status) for status in series_proto.statuses.series],
        dtype=MetricStatus,
    )


def _unpack_metrics_data(
    metrics_data: mp.MetricsData,
    id_to_unpacked_metrics_data: dict[uuid.UUID, BaseMetricsData],
) -> None:
    data_id = _unpack_uuid(metrics_data.metrics_data_id.id)
    unpacked: BaseMetricsData
    if metrics_data.data_type == mp.EXTERNAL_FILE_DATA_TYPE:
        unpacked = ExternalFileMetricsData(
            name=metrics_data.name, filename=metrics_data.external_file.path
        )
    elif metrics_data.is_per_category:
        assert metrics_data.WhichOneof("data") == "series_per_category"

        category_to_series = {}
        for category in metrics_data.category_names:
            series_to_unpack = metrics_data.series_per_category.category_to_series[
                category
            ]
            category_to_series[category] = _unpack_series(series_to_unpack)

        index = (
            id_to_unpacked_metrics_data[_unpack_uuid(metrics_data.index_data_id.id)]
            if metrics_data.is_indexed
            else None
        )
        index = cast(Optional[GroupedMetricsData], index)
        unpacked = GroupedMetricsData(
            name=metrics_data.name,
            category_to_series=category_to_series,
            unit=metrics_data.unit,
            index_data=index,
        )

    else:
        assert metrics_data.WhichOneof("data") == "series"
        index = (
            id_to_unpacked_metrics_data[_unpack_uuid(metrics_data.index_data_id.id)]
            if metrics_data.is_indexed
            else None
        )
        index = cast(Optional[SeriesMetricsData], index)
        unpacked = SeriesMetricsData(
            name=metrics_data.name,
            series=_unpack_series(metrics_data.series),
            unit=metrics_data.unit,
            index_data=index,
        )
    unpacked.id = data_id

    id_to_unpacked_metrics_data[data_id] = unpacked


def _unpack_metric(
    metric: mp.Metric,
    id_to_unpacked_metrics_data: dict[uuid.UUID, BaseMetricsData],
    id_to_unpacked_metrics: dict[uuid.UUID, Metric],
) -> Metric:
    unpacked = Metric.unpack_common_fields(metric)
    unpackers: dict[type[Any], Callable] = {
        DoubleSummaryMetric: _unpack_double_summary_metric,
        DoubleOverTimeMetric: _unpack_double_over_time_metric,
        LinePlotMetric: _unpack_line_plot_metric,
        BarChartMetric: _unpack_bar_chart_metric,
        StatesOverTimeMetric: _unpack_states_over_time_metric,
        HistogramMetric: _unpack_histogram_metric,
        ScalarMetric: _unpack_scalar_metric,
        PlotlyMetric: _unpack_plotly_metric,
        ImageMetric: _unpack_image_metric,
        ImageListMetric: _unpack_image_list_metric,
        TextMetric: _unpack_text_metric,
        BatchwiseBarChartMetric: _unpack_batchwise_bar_chart_metric,
    }
    unpacker: Callable = unpackers[type(unpacked)]
    unpacker(metric, unpacked, id_to_unpacked_metrics_data)
    id_to_unpacked_metrics[unpacked.id] = unpacked
    return unpacked


def _unpack_double_failure_definition(
    failure_definition: mp.DoubleFailureDefinition,
) -> DoubleFailureDefinition:
    return DoubleFailureDefinition(
        fails_above=(
            failure_definition.fails_above
            if failure_definition.HasField("fails_above")
            else None
        ),
        fails_below=(
            failure_definition.fails_below
            if failure_definition.HasField("fails_below")
            else None
        ),
    )


def _unpack_double_summary_metric(
    metric: mp.Metric,
    unpacked: DoubleSummaryMetric,
    id_to_unpacked_metrics_data: dict[uuid.UUID, MetricsData],
) -> None:
    values = metric.metric_values.double_metric_values

    data = id_to_unpacked_metrics_data[_unpack_uuid(values.value_data_id.id)]
    status = id_to_unpacked_metrics_data[_unpack_uuid(values.status_data_id.id)]

    unpacked.with_value_data(data).with_status_data(status)

    index_case = values.WhichOneof("index")
    if index_case == "series_index":
        unpacked.with_index(values.series_index)
    elif index_case == "timestamp_index":
        unpacked.with_index(Timestamp.unpack(values.timestamp_index))
    elif index_case == "uuid_index":
        unpacked.with_index(_unpack_uuid(values.uuid_index))
    elif index_case == "string_index":
        unpacked.with_index(values.string_index)

    unpacked.with_failure_definition(
        _unpack_double_failure_definition(values.failure_definition)
    )


def _unpack_double_over_time_metric(
    metric: mp.Metric,
    unpacked: DoubleOverTimeMetric,
    id_to_unpacked_metrics_data: dict[uuid.UUID, MetricsData],
) -> None:
    values = metric.metric_values.double_over_time_metric_values

    for i, (data_id, status_id) in enumerate(
        zip(values.doubles_over_time_data_id, values.statuses_over_time_data_id)
    ):
        name = values.legend_series_names[i] if values.legend_series_names else None
        data = id_to_unpacked_metrics_data[_unpack_uuid(data_id.id)]
        status = id_to_unpacked_metrics_data[_unpack_uuid(status_id.id)]
        unpacked.append_doubles_over_time_data(
            data, name
        ).append_statuses_over_time_data(status)

    failure_definitions = []
    for failure_definition in values.failure_definition:
        failure_definitions.append(
            _unpack_double_failure_definition(failure_definition)
        )

    unpacked.with_start_time(Timestamp.unpack(values.start_time)).with_end_time(
        Timestamp.unpack(values.end_time)
    ).with_y_axis_name(values.y_axis_name).with_failure_definitions(failure_definitions)


def _unpack_line_plot_metric(
    metric: mp.Metric,
    unpacked: LinePlotMetric,
    id_to_unpacked_metrics_data: dict[uuid.UUID, MetricsData],
) -> None:
    values = metric.metric_values.line_plot_metric_values

    for i, (x_data_id, y_data_id, status_id) in enumerate(
        zip(values.x_doubles_data_id, values.y_doubles_data_id, values.statuses_data_id)
    ):
        name = values.legend_series_names[i] if values.legend_series_names else None
        x_data = id_to_unpacked_metrics_data[_unpack_uuid(x_data_id.id)]
        y_data = id_to_unpacked_metrics_data[_unpack_uuid(y_data_id.id)]
        status = id_to_unpacked_metrics_data[_unpack_uuid(status_id.id)]
        unpacked.append_series_data(x_data, y_data, name).append_statuses_data(status)

    unpacked.with_x_axis_name(values.x_axis_name).with_y_axis_name(values.y_axis_name)


def _unpack_bar_chart_metric(
    metric: mp.Metric,
    unpacked: BarChartMetric,
    id_to_unpacked_metrics_data: dict[uuid.UUID, MetricsData],
) -> None:
    values = metric.metric_values.bar_chart_metric_values

    for i, (data_id, status_id) in enumerate(
        zip(values.values_data_id, values.statuses_data_id)
    ):
        name = values.legend_series_names[i] if values.legend_series_names else None
        data = id_to_unpacked_metrics_data[_unpack_uuid(data_id.id)]
        status = id_to_unpacked_metrics_data[_unpack_uuid(status_id.id)]
        unpacked.append_values_data(data, name).append_statuses_data(status)

    unpacked.with_x_axis_name(values.x_axis_name).with_y_axis_name(
        values.y_axis_name
    ).with_stack_bars(values.stack_bars)


def _unpack_batchwise_bar_chart_metric(
    metric: mp.Metric,
    unpacked: BatchwiseBarChartMetric,
    id_to_unpacked_metrics_data: dict[uuid.UUID, MetricsData],
) -> None:
    values = metric.metric_values.batchwise_bar_chart_metric_values

    for i, (time_id, data_id, status_id) in enumerate(
        zip(values.times_data_id, values.values_data_id, values.statuses_data_id)
    ):
        category = values.categories[i]
        times = id_to_unpacked_metrics_data[_unpack_uuid(time_id.id)]
        data = id_to_unpacked_metrics_data[_unpack_uuid(data_id.id)]
        status = id_to_unpacked_metrics_data[_unpack_uuid(status_id.id)]
        unpacked.append_category_data(category, times, data, status)

    unpacked.with_x_axis_name(values.x_axis_name).with_y_axis_name(
        values.y_axis_name
    ).with_stack_bars(values.stack_bars).with_colors(values.colors).with_project_id(
        uuid.UUID(values.project_id.data)
    )


def _unpack_states_over_time_metric(
    metric: mp.Metric,
    unpacked: StatesOverTimeMetric,
    id_to_unpacked_metrics_data: dict[uuid.UUID, MetricsData],
) -> None:
    values = metric.metric_values.states_over_time_metric_values

    for i, (data_id, status_id) in enumerate(
        zip(values.states_over_time_data_id, values.statuses_over_time_data_id)
    ):
        name = values.legend_series_names[i] if values.legend_series_names else None
        data = id_to_unpacked_metrics_data[_unpack_uuid(data_id.id)]
        status = id_to_unpacked_metrics_data[_unpack_uuid(status_id.id)]
        unpacked.append_states_over_time_data(
            data, name
        ).append_statuses_over_time_data(status)
    unpacked.with_states_set(values.states_set).with_failure_states(
        values.failure_states
    )


def _unpack_histogram_metric(
    metric: mp.Metric,
    unpacked: HistogramMetric,
    id_to_unpacked_metrics_data: dict[uuid.UUID, MetricsData],
) -> None:
    values = metric.metric_values.histogram_metric_values
    data = id_to_unpacked_metrics_data[_unpack_uuid(values.values_data_id.id)]
    status = id_to_unpacked_metrics_data[_unpack_uuid(values.statuses_data_id.id)]
    unpacked.with_values_data(data).with_statuses_data(status)

    buckets = []
    for bucket in values.buckets:
        buckets.append(HistogramBucket(lower=bucket.lower, upper=bucket.upper))
    unpacked.with_buckets(buckets).with_lower_bound(
        values.lower_bound
    ).with_upper_bound(values.upper_bound).with_x_axis_name(values.x_axis_name)


def _unpack_scalar_metric(
    metric: mp.Metric, unpacked: ScalarMetric, _: dict[uuid.UUID, MetricsData]
) -> None:
    values = metric.metric_values.scalar_metric_values
    unpacked.with_value(values.value).with_failure_definition(
        _unpack_double_failure_definition(values.failure_definition)
    ).with_unit(values.unit)


def _unpack_plotly_metric(
    metric: mp.Metric, unpacked: PlotlyMetric, _: dict[uuid.UUID, MetricsData]
) -> None:
    plotly_data_struct = metric.metric_values.plotly_metric_values.json
    plotly_data = pjf.MessageToJson(plotly_data_struct)
    unpacked.with_plotly_data(plotly_data)


def _unpack_image_metric(
    metric: mp.Metric,
    unpacked: ImageMetric,
    id_to_unpacked_metrics_data: dict[uuid.UUID, BaseMetricsData],
) -> None:
    image_data = metric.metric_values.image_metric_values
    data = cast(
        ExternalFileMetricsData,
        id_to_unpacked_metrics_data[_unpack_uuid(image_data.image_data_id.id)],
    )
    unpacked.with_image_data(data)


def _unpack_image_list_metric(
    metric: mp.Metric,
    unpacked: ImageListMetric,
    id_to_unpacked_metrics_data: dict[uuid.UUID, BaseMetricsData],
) -> None:
    image_list_data = metric.metric_values.image_list_metric_values
    data = []
    for image_data_id in image_list_data.image_data_ids:
        data.append(
            cast(
                ExternalFileMetricsData,
                id_to_unpacked_metrics_data[_unpack_uuid(image_data_id.id)],
            )
        )
    unpacked.with_image_list_data(data)


def _unpack_text_metric(
    metric: mp.Metric, unpacked: TextMetric, _: dict[uuid.UUID, MetricsData]
) -> None:
    text_metric_text = metric.metric_values.text_metric_values.text
    unpacked.with_text(text_metric_text)


def _unpack_event(
    msg: mp.Event, id_to_unpacked_metrics: dict[uuid.UUID, Metric]
) -> Event:
    unpacked = Event(name=msg.name)
    unpacked.id = uuid.UUID(msg.event_id.id.data)
    unpacked.description = msg.description
    unpacked.status = MetricStatus(msg.status)
    unpacked.importance = MetricImportance(msg.importance)
    # unpack the timestamp as either relative or absolute
    if msg.timestamp_type == mp.RELATIVE_TIMESTAMP:
        unpacked.with_relative_timestamp(Timestamp.unpack(msg.timestamp))
    else:
        unpacked.with_absolute_timestamp(Timestamp.unpack(msg.timestamp))

    unpacked.with_tags(msg.tags)
    # build the list of metrics:
    metrics = []
    for metric_id in msg.metrics:
        metrics.append(id_to_unpacked_metrics[_unpack_uuid(metric_id.id)])
    unpacked.with_metrics(metrics)

    return unpacked
