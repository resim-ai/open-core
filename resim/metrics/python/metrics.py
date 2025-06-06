# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""This module contains data classes used to represent metrics and metrics data
in a way that is significantly nicer to use than the raw protobuf API. In
particular, users don't have to think about IDs and certain other variants are
guaranteed when using this interface. This interface uses a Fluent API to easily
create metrics of different types.
"""

from __future__ import annotations

import dataclasses
import uuid
from abc import ABC, abstractmethod
from collections import defaultdict
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    List,
    Optional,
    Set,
    Type,
    TypeAlias,
    TypeVar,
    Union,
)

import numpy as np
from google.protobuf.json_format import Parse
from google.protobuf.struct_pb2 import Struct

import resim.metrics.proto.metrics_pb2 as metrics_proto
from resim.metrics.python.metrics_utils import (
    DoubleFailureDefinition,
    HistogramBucket,
    MetricImportance,
    MetricStatus,
    ResimMetricsOutput,
    Tag,
    Timestamp,
    TimestampType,
    pack_series_to_proto,
    pack_uuid_to_metric_id,
    pack_uuid_to_proto,
)

# ---------------------
# Metric representation
# ---------------------

MetricT = TypeVar("MetricT", bound="Metric")


T = TypeVar("T")


def metric_dataclass(cls: Type[T]) -> Type[T]:
    return dataclasses.dataclass(init=False, kw_only=True, repr=True, eq=False)(cls)


@metric_dataclass
class Metric(ABC, Generic[MetricT]):
    id: uuid.UUID
    name: str
    description: Optional[str]

    status: Optional[MetricStatus]
    importance: Optional[MetricImportance]

    should_display: Optional[bool]
    blocking: Optional[bool]
    parent_job_id: Optional[uuid.UUID]
    order: Optional[float]

    event_metric: Optional[bool]

    kv_tags: Optional[List[Tag]]

    @abstractmethod
    def __init__(
        self: Metric[MetricT],
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        should_display: Optional[bool] = None,
        blocking: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
    ):
        assert name is not None
        self.id = uuid.uuid4()
        self.name = name
        self.description = description
        self.status = status
        self.importance = importance
        self.should_display = should_display
        self.blocking = blocking
        self.parent_job_id = parent_job_id
        self.order = order
        self.event_metric = event_metric
        self.kv_tags = tags

    def __eq__(self: MetricT, __value: object) -> bool:
        if not isinstance(__value, type(self)):
            return False

        assert self.id is not None and __value.id is not None, (
            "Cannot compare values without valid ids"
        )

        return self.id == __value.id

    def with_description(self: MetricT, description: str) -> MetricT:
        self.description = description
        return self

    def with_status(self: MetricT, status: MetricStatus) -> MetricT:
        self.status = status
        return self

    def with_importance(self: MetricT, importance: MetricImportance) -> MetricT:
        self.importance = importance
        return self

    def with_should_display(self: MetricT, should_display: bool) -> MetricT:
        self.should_display = should_display
        return self

    def with_blocking(self: MetricT, blocking: bool) -> MetricT:
        self.blocking = blocking
        return self

    def with_tag(self: MetricT, key: str, value: str) -> MetricT:
        if self.kv_tags is None:
            self.kv_tags = []
        self.kv_tags.append(Tag(key, value))
        return self

    def is_event_metric(self: MetricT) -> MetricT:
        self.event_metric = True
        return self

    @abstractmethod
    def pack(self: MetricT) -> metrics_proto.Metric:
        msg = metrics_proto.Metric()

        msg.metric_id.id.CopyFrom(pack_uuid_to_proto(self.id))
        msg.name = self.name

        if self.description is not None:
            msg.description = self.description

        if self.status is not None:
            msg.status = self.status.value

        if self.importance is not None:
            msg.importance = self.importance.value

        if self.should_display is not None:
            msg.should_display = self.should_display

        if self.blocking is not None:
            msg.blocking = self.blocking

        if self.parent_job_id is not None:
            msg.job_id.id.CopyFrom(pack_uuid_to_proto(self.parent_job_id))

        if self.order is not None:
            msg.order = self.order

        if self.kv_tags is not None:
            for tag in self.kv_tags:
                msg.tags.append(tag.pack())

        if self.event_metric is not None:
            msg.event_metric = self.event_metric

        return msg

    @classmethod
    def unpack_common_fields(cls, msg: metrics_proto.Metric) -> Metric[Any]:
        unpacked = cls._unpacked_metric_type(msg)
        unpacked.id = uuid.UUID(msg.metric_id.id.data)
        unpacked.description = msg.description
        unpacked.status = MetricStatus(msg.status)
        unpacked.importance = MetricImportance(msg.importance)
        if msg.HasField("should_display"):
            unpacked.should_display = msg.should_display
        else:
            unpacked.should_display = None

        if msg.HasField("blocking"):
            unpacked.blocking = msg.blocking
        else:
            unpacked.blocking = None

        if msg.HasField("job_id"):
            unpacked.parent_job_id = uuid.UUID(msg.job_id.id.data)
        else:
            unpacked.parent_job_id = None

        if msg.HasField("order"):
            unpacked.order = msg.order
        else:
            unpacked.order = None

        if len(msg.tags) > 0:
            unpacked.kv_tags = []
            for tag in msg.tags:
                unpacked.kv_tags.append(Tag.unpack(tag))
        else:
            unpacked.kv_tags = None

        if msg.HasField("event_metric"):
            unpacked.event_metric = msg.event_metric
        else:
            unpacked.event_metric = None

        return unpacked

    @classmethod
    def _unpacked_metric_type(cls, msg: metrics_proto.Metric) -> Metric[Any]:
        if msg.type == metrics_proto.MetricType.Value("NO_METRIC_TYPE"):
            raise ValueError("Cannot unpack with no metric type")
        if msg.type == metrics_proto.MetricType.Value("DOUBLE_SUMMARY_METRIC_TYPE"):
            unpacked: Metric[Any] = DoubleSummaryMetric(name=msg.name)
        elif msg.type == metrics_proto.MetricType.Value("DOUBLE_OVER_TIME_METRIC_TYPE"):
            unpacked = DoubleOverTimeMetric(name=msg.name)
        elif msg.type == metrics_proto.MetricType.Value("LINE_PLOT_METRIC_TYPE"):
            unpacked = LinePlotMetric(name=msg.name)
        elif msg.type == metrics_proto.MetricType.Value("BAR_CHART_METRIC_TYPE"):
            unpacked = BarChartMetric(name=msg.name)
        elif msg.type == metrics_proto.MetricType.Value(
            "BATCHWISE_BAR_CHART_METRIC_TYPE"
        ):
            unpacked = BatchwiseBarChartMetric(name=msg.name)
        elif msg.type == metrics_proto.MetricType.Value("STATES_OVER_TIME_METRIC_TYPE"):
            unpacked = StatesOverTimeMetric(name=msg.name)
        elif msg.type == metrics_proto.MetricType.Value("HISTOGRAM_METRIC_TYPE"):
            unpacked = HistogramMetric(name=msg.name)
        elif msg.type == metrics_proto.MetricType.Value("SCALAR_METRIC_TYPE"):
            unpacked = ScalarMetric(name=msg.name)
        elif msg.type == metrics_proto.MetricType.Value("PLOTLY_METRIC_TYPE"):
            unpacked = PlotlyMetric(name=msg.name)
        elif msg.type == metrics_proto.MetricType.Value("IMAGE_METRIC_TYPE"):
            unpacked = ImageMetric(name=msg.name)
        elif msg.type == metrics_proto.MetricType.Value("IMAGE_LIST_METRIC_TYPE"):
            unpacked = ImageListMetric(name=msg.name)
        elif msg.type == metrics_proto.MetricType.Value("TEXT_METRIC_TYPE"):
            unpacked = TextMetric(name=msg.name)
        else:
            raise ValueError("Invalid metric type")

        return unpacked

    @abstractmethod
    def recursively_pack_into(self, metrics_output: ResimMetricsOutput) -> None:
        raise NotImplementedError()


@metric_dataclass
class ScalarMetric(Metric["ScalarMetric"]):
    value: Optional[float]
    failure_definition: Optional[DoubleFailureDefinition]
    unit: Optional[str]

    def __init__(
        self: ScalarMetric,
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        should_display: Optional[bool] = None,
        blocking: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
        value: Optional[float] = None,
        failure_definition: Optional[DoubleFailureDefinition] = None,
        unit: Optional[str] = None,
    ):
        super().__init__(
            name=name,
            description=description,
            status=status,
            importance=importance,
            blocking=blocking,
            should_display=should_display,
            parent_job_id=parent_job_id,
            order=order,
            event_metric=event_metric,
            tags=tags,
        )
        self.value = value
        self.failure_definition = failure_definition
        self.unit = unit

    def with_value(self: ScalarMetric, value: float) -> ScalarMetric:
        self.value = value
        return self

    def with_unit(self: ScalarMetric, unit: str) -> ScalarMetric:
        self.unit = unit
        return self

    def with_failure_definition(
        self: ScalarMetric, failure_definition: DoubleFailureDefinition
    ) -> ScalarMetric:
        self.failure_definition = failure_definition
        return self

    def pack(self: ScalarMetric) -> metrics_proto.Metric:
        msg = super().pack()
        msg.type = metrics_proto.MetricType.Value("SCALAR_METRIC_TYPE")

        metric_values = msg.metric_values.scalar_metric_values
        if self.value is not None:
            metric_values.value = self.value

        if self.failure_definition is not None:
            metric_values.failure_definition.CopyFrom(self.failure_definition.pack())

        if self.unit is not None:
            metric_values.unit = self.unit

        return msg

    def recursively_pack_into(
        self: ScalarMetric, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.job_level_metrics.metrics.extend([self.pack()])


@metric_dataclass
class DoubleOverTimeMetric(Metric["DoubleOverTimeMetric"]):
    doubles_over_time_data: List[MetricsData]
    statuses_over_time_data: List[MetricsData]

    failure_definitions: List[DoubleFailureDefinition]

    start_time: Optional[Timestamp]
    end_time: Optional[Timestamp]

    y_axis_name: Optional[str]
    legend_series_names: List[Optional[str]]

    def __init__(
        self: DoubleOverTimeMetric,
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        blocking: Optional[bool] = None,
        should_display: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
        doubles_over_time_data: Optional[List[MetricsData]] = None,
        statuses_over_time_data: Optional[List[MetricsData]] = None,
        failure_definitions: Optional[List[DoubleFailureDefinition]] = None,
        start_time: Optional[Timestamp] = None,
        end_time: Optional[Timestamp] = None,
        y_axis_name: Optional[str] = None,
        legend_series_names: Optional[List[Optional[str]]] = None,
    ):
        super().__init__(
            name=name,
            description=description,
            status=status,
            importance=importance,
            blocking=blocking,
            should_display=should_display,
            parent_job_id=parent_job_id,
            order=order,
            event_metric=event_metric,
            tags=tags,
        )
        if doubles_over_time_data is None:
            self.doubles_over_time_data = []
        else:
            self.doubles_over_time_data = doubles_over_time_data

        if statuses_over_time_data is None:
            self.statuses_over_time_data = []
        else:
            self.statuses_over_time_data = statuses_over_time_data

        if failure_definitions is None:
            self.failure_definitions = []
        else:
            self.failure_definitions = failure_definitions

        self.start_time = start_time
        self.end_time = end_time
        self.y_axis_name = y_axis_name

        if legend_series_names is None:
            self.legend_series_names = []
        else:
            self.legend_series_names = legend_series_names

    def with_doubles_over_time_data(
        self: DoubleOverTimeMetric, doubles_over_time_data: List[MetricsData]
    ) -> DoubleOverTimeMetric:
        self.doubles_over_time_data = doubles_over_time_data
        return self

    def with_statuses_over_time_data(
        self: DoubleOverTimeMetric, statuses_over_time_data: List[MetricsData]
    ) -> DoubleOverTimeMetric:
        self.statuses_over_time_data = statuses_over_time_data
        return self

    def append_doubles_over_time_data(
        self: DoubleOverTimeMetric,
        double_over_time_data_element: MetricsData,
        legend_series_name: Optional[str] = None,
    ) -> DoubleOverTimeMetric:
        self.doubles_over_time_data.append(double_over_time_data_element)
        self.legend_series_names.append(legend_series_name)
        return self

    def append_statuses_over_time_data(
        self: DoubleOverTimeMetric, statuses_over_time_data_element: MetricsData
    ) -> DoubleOverTimeMetric:
        self.statuses_over_time_data.append(statuses_over_time_data_element)
        return self

    def with_failure_definitions(
        self: DoubleOverTimeMetric, failure_definitions: List[DoubleFailureDefinition]
    ) -> DoubleOverTimeMetric:
        self.failure_definitions = failure_definitions
        return self

    def with_start_time(
        self: DoubleOverTimeMetric, start_time: Timestamp
    ) -> DoubleOverTimeMetric:
        self.start_time = start_time
        return self

    def with_end_time(
        self: DoubleOverTimeMetric, end_time: Timestamp
    ) -> DoubleOverTimeMetric:
        self.end_time = end_time
        return self

    def with_y_axis_name(
        self: DoubleOverTimeMetric, y_axis_name: str
    ) -> DoubleOverTimeMetric:
        self.y_axis_name = y_axis_name
        return self

    def with_legend_series_names(
        self: DoubleOverTimeMetric, legend_series_names: List[Optional[str]]
    ) -> DoubleOverTimeMetric:
        self.legend_series_names = legend_series_names
        return self

    def pack(self: DoubleOverTimeMetric) -> metrics_proto.Metric:
        msg = super().pack()
        msg.type = metrics_proto.MetricType.Value("DOUBLE_OVER_TIME_METRIC_TYPE")

        metric_values = msg.metric_values.double_over_time_metric_values

        for data in self.doubles_over_time_data:
            id_msg = metrics_proto.MetricsDataId()
            id_msg.id.CopyFrom(pack_uuid_to_proto(data.id))
            metric_values.doubles_over_time_data_id.extend([id_msg])

        for data in self.statuses_over_time_data:
            id_msg = metrics_proto.MetricsDataId()
            id_msg.id.CopyFrom(pack_uuid_to_proto(data.id))
            metric_values.statuses_over_time_data_id.extend([id_msg])

        metric_values.failure_definition.extend(
            [d.pack() for d in self.failure_definitions]
        )

        if self.start_time is not None:
            metric_values.start_time.CopyFrom(self.start_time.pack())

        if self.end_time is not None:
            metric_values.end_time.CopyFrom(self.end_time.pack())

        if self.y_axis_name is not None:
            metric_values.y_axis_name = self.y_axis_name

        for i, name in enumerate(self.legend_series_names):
            if name is not None:
                metric_values.legend_series_names.extend([name])
            else:
                metric_values.legend_series_names.extend(
                    [self.doubles_over_time_data[i].name]
                )

        return msg

    def recursively_pack_into(
        self: DoubleOverTimeMetric, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.job_level_metrics.metrics.extend([self.pack()])

        for data in self.doubles_over_time_data:
            data.recursively_pack_into(metrics_output)

        for data in self.statuses_over_time_data:
            data.recursively_pack_into(metrics_output)


@metric_dataclass
class StatesOverTimeMetric(Metric["StatesOverTimeMetric"]):
    states_over_time_data: List[MetricsData]
    statuses_over_time_data: List[MetricsData]

    states_set: Optional[Set[str]]
    failure_states: Optional[Set[str]]

    legend_series_names: List[Optional[str]]

    def __init__(
        self: StatesOverTimeMetric,
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        blocking: Optional[bool] = None,
        should_display: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
        states_over_time_data: Optional[List[MetricsData]] = None,
        statuses_over_time_data: Optional[List[MetricsData]] = None,
        states_set: Optional[Set[str]] = None,
        failure_states: Optional[Set[str]] = None,
        legend_series_names: Optional[List[Optional[str]]] = None,
    ):
        super().__init__(
            name=name,
            description=description,
            status=status,
            importance=importance,
            blocking=blocking,
            should_display=should_display,
            parent_job_id=parent_job_id,
            order=order,
            event_metric=event_metric,
            tags=tags,
        )
        if states_over_time_data is None:
            self.states_over_time_data = []
        else:
            self.states_over_time_data = states_over_time_data

        if statuses_over_time_data is None:
            self.statuses_over_time_data = []
        else:
            self.statuses_over_time_data = statuses_over_time_data

        self.states_set = states_set
        self.failure_states = failure_states

        if legend_series_names is None:
            self.legend_series_names = []
        else:
            self.legend_series_names = legend_series_names

    def with_statuses_over_time_data(
        self: StatesOverTimeMetric, statuses_over_time_data: List[MetricsData]
    ) -> StatesOverTimeMetric:
        self.statuses_over_time_data = statuses_over_time_data
        return self

    def with_states_over_time_data(
        self: StatesOverTimeMetric, states_over_time_data: List[MetricsData]
    ) -> StatesOverTimeMetric:
        self.states_over_time_data = states_over_time_data
        return self

    def with_states_over_time_series(
        self: StatesOverTimeMetric,
        states_over_time_series: Dict[str, np.ndarray],
        units: Optional[Dict[str, str]] = None,
        indices: Optional[Dict[str, np.ndarray]] = None,
        legend_series_names: Optional[Dict[str, str]] = None,
    ) -> StatesOverTimeMetric:
        for name, series in states_over_time_series.items():
            unit, index_data = None, None
            if indices is not None and name in indices:
                index = indices[name]
                index_data = SeriesMetricsData(f"{name}_series_index", index)

            if units is not None and name in units:
                unit = units[name]

            data = SeriesMetricsData(name, series, unit, index_data)

            self.states_over_time_data.append(data)

            if legend_series_names is not None and name in legend_series_names:
                self.legend_series_names.append(legend_series_names[name])
            else:
                self.legend_series_names.append(None)

        return self

    def append_states_over_time_data(
        self: StatesOverTimeMetric,
        states_over_time_data_element: MetricsData,
        legend_series_name: Optional[str] = None,
    ) -> StatesOverTimeMetric:
        self.states_over_time_data.append(states_over_time_data_element)
        self.legend_series_names.append(legend_series_name)
        return self

    def append_statuses_over_time_data(
        self: StatesOverTimeMetric, statuses_over_time_data_element: MetricsData
    ) -> StatesOverTimeMetric:
        self.statuses_over_time_data.append(statuses_over_time_data_element)
        return self

    def with_states_set(
        self: StatesOverTimeMetric, states_set: Set[str]
    ) -> StatesOverTimeMetric:
        self.states_set = states_set
        return self

    def with_failure_states(
        self: StatesOverTimeMetric, failure_states: Set[str]
    ) -> StatesOverTimeMetric:
        self.failure_states = failure_states
        return self

    def with_legend_series_names(
        self: StatesOverTimeMetric, legend_series_names: List[Optional[str]]
    ) -> StatesOverTimeMetric:
        self.legend_series_names = legend_series_names
        return self

    def pack(self: StatesOverTimeMetric) -> metrics_proto.Metric:
        msg = super().pack()
        msg.type = metrics_proto.MetricType.Value("STATES_OVER_TIME_METRIC_TYPE")

        metric_values = msg.metric_values.states_over_time_metric_values

        for data in self.states_over_time_data:
            id_msg = metrics_proto.MetricsDataId()
            id_msg.id.CopyFrom(pack_uuid_to_proto(data.id))
            metric_values.states_over_time_data_id.extend([id_msg])

        for data in self.statuses_over_time_data:
            id_msg = metrics_proto.MetricsDataId()
            id_msg.id.CopyFrom(pack_uuid_to_proto(data.id))
            metric_values.statuses_over_time_data_id.extend([id_msg])

        if self.states_set is not None:
            metric_values.states_set.extend(list(self.states_set))

        if self.failure_states is not None:
            metric_values.failure_states.extend(list(self.failure_states))

        for i, name in enumerate(self.legend_series_names):
            if name is not None:
                metric_values.legend_series_names.extend([name])
            else:
                metric_values.legend_series_names.extend(
                    [self.states_over_time_data[i].name]
                )

        return msg

    def recursively_pack_into(
        self: StatesOverTimeMetric, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.job_level_metrics.metrics.extend([self.pack()])

        for data in self.states_over_time_data:
            data.recursively_pack_into(metrics_output)

        for data in self.statuses_over_time_data:
            data.recursively_pack_into(metrics_output)


@metric_dataclass
class LinePlotMetric(Metric["LinePlotMetric"]):
    x_doubles_data: List[MetricsData]
    y_doubles_data: List[MetricsData]
    statuses_data: List[MetricsData]

    x_axis_name: Optional[str]
    y_axis_name: Optional[str]

    legend_series_names: List[Optional[str]]

    def __init__(
        self: LinePlotMetric,
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        blocking: Optional[bool] = None,
        should_display: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
        x_doubles_data: Optional[List[MetricsData]] = None,
        y_doubles_data: Optional[List[MetricsData]] = None,
        statuses_data: Optional[List[MetricsData]] = None,
        x_axis_name: Optional[str] = None,
        y_axis_name: Optional[str] = None,
        legend_series_names: Optional[List[Optional[str]]] = None,
    ):
        super().__init__(
            name=name,
            description=description,
            status=status,
            importance=importance,
            blocking=blocking,
            should_display=should_display,
            parent_job_id=parent_job_id,
            order=order,
            event_metric=event_metric,
            tags=tags,
        )
        if x_doubles_data is None:
            self.x_doubles_data = []
        else:
            self.x_doubles_data = x_doubles_data

        if y_doubles_data is None:
            self.y_doubles_data = []
        else:
            self.y_doubles_data = y_doubles_data

        if statuses_data is None:
            self.statuses_data = []
        else:
            self.statuses_data = statuses_data

        self.x_axis_name = x_axis_name
        self.y_axis_name = y_axis_name

        if legend_series_names is None:
            self.legend_series_names = []
        else:
            self.legend_series_names = legend_series_names

    def append_series_data(
        self: LinePlotMetric,
        x_doubles_data: MetricsData,
        y_doubles_data: MetricsData,
        legend_series_name: Optional[str] = None,
    ) -> LinePlotMetric:
        self.x_doubles_data.append(x_doubles_data)
        self.y_doubles_data.append(y_doubles_data)
        self.legend_series_names.append(legend_series_name)

        return self

    def append_statuses_data(
        self: LinePlotMetric, statuses_data: MetricsData
    ) -> LinePlotMetric:
        self.statuses_data.append(statuses_data)
        return self

    def with_x_axis_name(self: LinePlotMetric, x_axis_name: str) -> LinePlotMetric:
        self.x_axis_name = x_axis_name
        return self

    def with_y_axis_name(self: LinePlotMetric, y_axis_name: str) -> LinePlotMetric:
        self.y_axis_name = y_axis_name
        return self

    def with_legend_series_names(
        self: LinePlotMetric, legend_series_names: List[Optional[str]]
    ) -> LinePlotMetric:
        self.legend_series_names = legend_series_names
        return self

    def pack(self: LinePlotMetric) -> metrics_proto.Metric:
        msg = super().pack()
        msg.type = metrics_proto.MetricType.Value("LINE_PLOT_METRIC_TYPE")

        metric_values = msg.metric_values.line_plot_metric_values

        for data in self.x_doubles_data:
            id_msg = metrics_proto.MetricsDataId()
            id_msg.id.CopyFrom(pack_uuid_to_proto(data.id))
            metric_values.x_doubles_data_id.extend([id_msg])

        for data in self.y_doubles_data:
            id_msg = metrics_proto.MetricsDataId()
            id_msg.id.CopyFrom(pack_uuid_to_proto(data.id))
            metric_values.y_doubles_data_id.extend([id_msg])

        for data in self.statuses_data:
            id_msg = metrics_proto.MetricsDataId()
            id_msg.id.CopyFrom(pack_uuid_to_proto(data.id))
            metric_values.statuses_data_id.extend([id_msg])

        if self.x_axis_name is not None:
            metric_values.x_axis_name = self.x_axis_name

        if self.y_axis_name is not None:
            metric_values.y_axis_name = self.y_axis_name

        for i, name in enumerate(self.legend_series_names):
            if name is not None:
                metric_values.legend_series_names.extend([name])
            else:
                metric_values.legend_series_names.extend([self.y_doubles_data[i].name])

        return msg

    def recursively_pack_into(
        self: LinePlotMetric, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.job_level_metrics.metrics.extend([self.pack()])

        for data in self.x_doubles_data:
            data.recursively_pack_into(metrics_output)

        for data in self.y_doubles_data:
            data.recursively_pack_into(metrics_output)

        for data in self.statuses_data:
            data.recursively_pack_into(metrics_output)


@metric_dataclass
class BarChartMetric(Metric["BarChartMetric"]):
    values_data: List[MetricsData]
    statuses_data: List[MetricsData]
    legend_series_names: List[Optional[str]]
    x_axis_name: Optional[str]
    y_axis_name: Optional[str]
    stack_bars: Optional[bool]

    def __init__(
        self: BarChartMetric,
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        blocking: Optional[bool] = None,
        should_display: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
        values_data: Optional[List[MetricsData]] = None,
        statuses_data: Optional[List[MetricsData]] = None,
        legend_series_names: Optional[List[Optional[str]]] = None,
        x_axis_name: Optional[str] = None,
        y_axis_name: Optional[str] = None,
        stack_bars: Optional[bool] = None,
    ):
        super().__init__(
            name=name,
            description=description,
            status=status,
            importance=importance,
            blocking=blocking,
            should_display=should_display,
            parent_job_id=parent_job_id,
            order=order,
            event_metric=event_metric,
            tags=tags,
        )

        if values_data is None:
            self.values_data = []
        else:
            self.values_data = values_data

        if statuses_data is None:
            self.statuses_data = []
        else:
            self.statuses_data = statuses_data

        if legend_series_names is None:
            self.legend_series_names = []
        else:
            self.legend_series_names = legend_series_names

        self.x_axis_name = x_axis_name
        self.y_axis_name = y_axis_name
        self.stack_bars = stack_bars

    def append_values_data(
        self: BarChartMetric,
        values_data_element: MetricsData,
        legend_series_name: Optional[str] = None,
    ) -> BarChartMetric:
        self.values_data.append(values_data_element)
        self.legend_series_names.append(legend_series_name)
        return self

    def append_statuses_data(
        self: BarChartMetric, statuses_data_element: MetricsData
    ) -> BarChartMetric:
        self.statuses_data.append(statuses_data_element)
        return self

    def with_x_axis_name(self: BarChartMetric, x_axis_name: str) -> BarChartMetric:
        self.x_axis_name = x_axis_name
        return self

    def with_y_axis_name(self: BarChartMetric, y_axis_name: str) -> BarChartMetric:
        self.y_axis_name = y_axis_name
        return self

    def with_legend_series_names(
        self: BarChartMetric, legend_series_names: List[Optional[str]]
    ) -> BarChartMetric:
        self.legend_series_names = legend_series_names
        return self

    def with_stack_bars(self, stack_bars: bool) -> BarChartMetric:
        self.stack_bars = stack_bars
        return self

    def pack(self: BarChartMetric) -> metrics_proto.Metric:
        msg = super().pack()
        msg.type = metrics_proto.MetricType.Value("BAR_CHART_METRIC_TYPE")

        metric_values = msg.metric_values.bar_chart_metric_values

        for data in self.values_data:
            id_msg = metrics_proto.MetricsDataId()
            id_msg.id.CopyFrom(pack_uuid_to_proto(data.id))
            metric_values.values_data_id.extend([id_msg])

        for data in self.statuses_data:
            id_msg = metrics_proto.MetricsDataId()
            id_msg.id.CopyFrom(pack_uuid_to_proto(data.id))
            metric_values.statuses_data_id.extend([id_msg])

        if self.x_axis_name is not None:
            metric_values.x_axis_name = self.x_axis_name

        if self.y_axis_name is not None:
            metric_values.y_axis_name = self.y_axis_name

        if self.stack_bars is not None:
            metric_values.stack_bars = self.stack_bars

        for i, name in enumerate(self.legend_series_names):
            if name is not None:
                metric_values.legend_series_names.extend([name])
            else:
                metric_values.legend_series_names.extend([self.values_data[i].name])

        return msg

    def recursively_pack_into(
        self: BarChartMetric, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.job_level_metrics.metrics.extend([self.pack()])

        for data in self.values_data:
            data.recursively_pack_into(metrics_output)

        for data in self.statuses_data:
            data.recursively_pack_into(metrics_output)


@metric_dataclass
class BatchwiseBarChartMetric(Metric["BatchwiseBarChartMetric"]):
    times_data: List[MetricsData] = dataclasses.field(default_factory=list)
    values_data: List[MetricsData] = dataclasses.field(default_factory=list)
    statuses_data: List[MetricsData] = dataclasses.field(default_factory=list)
    categories: List[str] = dataclasses.field(default_factory=list)
    colors: List[str] = dataclasses.field(default_factory=list)
    project_id: Optional[uuid.UUID]
    x_axis_name: str
    y_axis_name: str
    stack_bars: bool

    def __init__(
        self: BatchwiseBarChartMetric,
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        blocking: Optional[bool] = None,
        should_display: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
        times_data: Optional[List[MetricsData]] = None,
        values_data: Optional[List[MetricsData]] = None,
        statuses_data: Optional[List[MetricsData]] = None,
        categories: Optional[List[str]] = None,
        colors: Optional[List[str]] = None,
        project_id: Optional[uuid.UUID] = None,
        x_axis_name: str = "",
        y_axis_name: str = "",
        stack_bars: bool = False,
    ):
        super().__init__(
            name=name,
            description=description,
            status=status,
            importance=importance,
            blocking=blocking,
            should_display=should_display,
            parent_job_id=parent_job_id,
            order=order,
            event_metric=event_metric,
            tags=tags,
        )
        if times_data is not None:
            self.times_data = times_data
        else:
            self.times_data = []
        if values_data is not None:
            self.values_data = values_data
        else:
            self.values_data = []
        if statuses_data is not None:
            self.statuses_data = statuses_data
        else:
            self.statuses_data = []
        if categories is not None:
            self.categories = categories
        else:
            self.categories = []
        if colors is not None:
            self.colors = colors
        else:
            self.colors = []
        self.project_id = project_id
        self.x_axis_name = x_axis_name
        self.y_axis_name = y_axis_name
        self.stack_bars = stack_bars

    def append_category_data(
        self: BatchwiseBarChartMetric,
        category: str,
        times_data: MetricsData,
        values_data: MetricsData,
        statuses_data: MetricsData,
    ) -> BatchwiseBarChartMetric:
        self.categories.append(category)
        self.times_data.append(times_data)
        self.values_data.append(values_data)
        self.statuses_data.append(statuses_data)

        return self

    def with_colors(
        self: BatchwiseBarChartMetric, colors: List[str]
    ) -> BatchwiseBarChartMetric:
        self.colors = colors
        return self

    def with_x_axis_name(
        self: BatchwiseBarChartMetric, x_axis_name: str
    ) -> BatchwiseBarChartMetric:
        self.x_axis_name = x_axis_name
        return self

    def with_y_axis_name(
        self: BatchwiseBarChartMetric, y_axis_name: str
    ) -> BatchwiseBarChartMetric:
        self.y_axis_name = y_axis_name
        return self

    def with_stack_bars(self, stack_bars: bool) -> BatchwiseBarChartMetric:
        self.stack_bars = stack_bars
        return self

    def with_project_id(self, project_id: uuid.UUID) -> BatchwiseBarChartMetric:
        self.project_id = project_id
        return self

    def pack(self: BatchwiseBarChartMetric) -> metrics_proto.Metric:
        msg = super().pack()
        msg.type = metrics_proto.MetricType.Value("BATCHWISE_BAR_CHART_METRIC_TYPE")
        metric_values = msg.metric_values.batchwise_bar_chart_metric_values

        assert self.project_id is not None

        if self.times_data is not None:
            for times_data in self.times_data:
                metric_values.times_data_id.add().id.CopyFrom(
                    pack_uuid_to_proto(times_data.id)
                )

        if self.values_data is not None:
            for values_data in self.values_data:
                metric_values.values_data_id.add().id.CopyFrom(
                    pack_uuid_to_proto(values_data.id)
                )

        if self.statuses_data is not None:
            for statuses_data in self.statuses_data:
                metric_values.statuses_data_id.add().id.CopyFrom(
                    pack_uuid_to_proto(statuses_data.id)
                )

        if self.categories is not None:
            metric_values.categories.extend(self.categories)

        if self.colors is not None:
            metric_values.colors.extend(self.colors)

        metric_values.project_id.data = str(self.project_id)
        metric_values.x_axis_name = self.x_axis_name
        metric_values.y_axis_name = self.y_axis_name
        metric_values.stack_bars = self.stack_bars

        return msg

    def recursively_pack_into(
        self: BatchwiseBarChartMetric, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.job_level_metrics.metrics.extend([self.pack()])
        if self.times_data is not None:
            for times_data in self.times_data:
                times_data.recursively_pack_into(metrics_output)

        if self.values_data is not None:
            for values_data in self.values_data:
                values_data.recursively_pack_into(metrics_output)

        if self.statuses_data is not None:
            for statuses_data in self.statuses_data:
                statuses_data.recursively_pack_into(metrics_output)


@metric_dataclass
class HistogramMetric(Metric["HistogramMetric"]):
    values_data: Optional[MetricsData]
    statuses_data: Optional[MetricsData]
    buckets: Optional[List[HistogramBucket]]
    lower_bound: Optional[float]
    upper_bound: Optional[float]

    def __init__(
        self: HistogramMetric,
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        blocking: Optional[bool] = None,
        should_display: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
        values_data: Optional[MetricsData] = None,
        statuses_data: Optional[MetricsData] = None,
        buckets: Optional[List[HistogramBucket]] = None,
        lower_bound: Optional[float] = None,
        upper_bound: Optional[float] = None,
        x_axis_name: Optional[str] = None,
    ):
        super().__init__(
            name=name,
            description=description,
            status=status,
            importance=importance,
            blocking=blocking,
            should_display=should_display,
            parent_job_id=parent_job_id,
            order=order,
            event_metric=event_metric,
            tags=tags,
        )

        self.values_data = values_data
        self.statuses_data = statuses_data
        self.buckets = buckets
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.x_axis_name = x_axis_name

    def with_values_data(
        self: HistogramMetric, values_data: MetricsData
    ) -> HistogramMetric:
        self.values_data = values_data
        return self

    def with_statuses_data(
        self: HistogramMetric, statuses_data: MetricsData
    ) -> HistogramMetric:
        self.statuses_data = statuses_data
        return self

    def with_buckets(
        self: HistogramMetric, buckets: List[HistogramBucket]
    ) -> HistogramMetric:
        self.buckets = buckets
        return self

    def with_lower_bound(self: HistogramMetric, lower_bound: float) -> HistogramMetric:
        self.lower_bound = lower_bound
        return self

    def with_upper_bound(self: HistogramMetric, upper_bound: float) -> HistogramMetric:
        self.upper_bound = upper_bound
        return self

    def with_x_axis_name(self: HistogramMetric, x_axis_name: str) -> HistogramMetric:
        self.x_axis_name = x_axis_name
        return self

    def pack(self: HistogramMetric) -> metrics_proto.Metric:
        msg = super().pack()
        msg.type = metrics_proto.MetricType.Value("HISTOGRAM_METRIC_TYPE")

        metric_values = msg.metric_values.histogram_metric_values

        if self.values_data is not None:
            metric_values.values_data_id.id.CopyFrom(
                pack_uuid_to_proto(self.values_data.id)
            )

        if self.statuses_data is not None:
            metric_values.statuses_data_id.id.CopyFrom(
                pack_uuid_to_proto(self.statuses_data.id)
            )

        if self.x_axis_name is not None:
            metric_values.x_axis_name = self.x_axis_name

        if self.lower_bound is not None:
            metric_values.lower_bound = self.lower_bound

        if self.upper_bound is not None:
            metric_values.upper_bound = self.upper_bound

        if self.buckets is not None:
            metric_values.buckets.extend([b.pack() for b in self.buckets])

        return msg

    def recursively_pack_into(
        self: HistogramMetric, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.job_level_metrics.metrics.extend([self.pack()])
        if self.values_data is not None:
            self.values_data.recursively_pack_into(metrics_output)

        if self.statuses_data is not None:
            self.statuses_data.recursively_pack_into(metrics_output)


# Indices are a simple union, as we don't bind the type
IndexType: TypeAlias = Union[int, str, Timestamp, uuid.UUID]


@metric_dataclass
class DoubleSummaryMetric(Metric["DoubleSummaryMetric"]):
    value_data: Optional[MetricsData]
    status_data: Optional[MetricsData]
    index: Optional[IndexType]
    failure_definition: Optional[DoubleFailureDefinition]

    def __init__(
        self: DoubleSummaryMetric,
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        blocking: Optional[bool] = None,
        should_display: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
        value_data: Optional[MetricsData] = None,
        status_data: Optional[MetricsData] = None,
        index: Optional[IndexType] = None,
        failure_definition: Optional[DoubleFailureDefinition] = None,
    ):
        super().__init__(
            name=name,
            description=description,
            status=status,
            importance=importance,
            blocking=blocking,
            should_display=should_display,
            parent_job_id=parent_job_id,
            order=order,
            event_metric=event_metric,
            tags=tags,
        )

        self.value_data = value_data
        self.status_data = status_data
        self.index = index
        self.failure_definition = failure_definition

    def with_value_data(
        self: DoubleSummaryMetric, value_data: MetricsData
    ) -> DoubleSummaryMetric:
        self.value_data = value_data
        return self

    def with_status_data(
        self: DoubleSummaryMetric, status_data: MetricsData
    ) -> DoubleSummaryMetric:
        self.status_data = status_data
        return self

    def with_index(self: DoubleSummaryMetric, index: IndexType) -> DoubleSummaryMetric:
        self.index = index
        return self

    def with_failure_definition(
        self: DoubleSummaryMetric, failure_definition: DoubleFailureDefinition
    ) -> DoubleSummaryMetric:
        self.failure_definition = failure_definition
        return self

    def pack(self: DoubleSummaryMetric) -> metrics_proto.Metric:
        msg = super().pack()
        msg.type = metrics_proto.MetricType.Value("DOUBLE_SUMMARY_METRIC_TYPE")

        metric_values = msg.metric_values.double_metric_values

        if self.value_data is not None:
            metric_values.value_data_id.id.CopyFrom(
                pack_uuid_to_proto(self.value_data.id)
            )

        if self.status_data is not None:
            metric_values.status_data_id.id.CopyFrom(
                pack_uuid_to_proto(self.status_data.id)
            )

        if self.index is not None:
            # int, str, Timestamp, uuid.UUID
            if isinstance(self.index, int):
                metric_values.series_index = self.index
            elif isinstance(self.index, str):
                metric_values.string_index = self.index
            elif isinstance(self.index, Timestamp):
                metric_values.timestamp_index.CopyFrom(self.index.pack())
            elif isinstance(self.index, uuid.UUID):
                metric_values.uuid_index.CopyFrom(pack_uuid_to_proto(self.index))
            else:
                raise ValueError(f"Packed invalid index type: {type(self.index)}")

        if self.failure_definition is not None:
            metric_values.failure_definition.CopyFrom(self.failure_definition.pack())

        return msg

    def recursively_pack_into(
        self: DoubleSummaryMetric, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.job_level_metrics.metrics.extend([self.pack()])

        if self.value_data is not None:
            self.value_data.recursively_pack_into(metrics_output)

        if self.status_data is not None:
            self.status_data.recursively_pack_into(metrics_output)


@metric_dataclass
class PlotlyMetric(Metric["PlotlyMetric"]):
    plotly_data: Optional[str]

    def __init__(
        self: PlotlyMetric,
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        blocking: Optional[bool] = None,
        should_display: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
        plotly_data: Optional[str] = None,
    ):
        super().__init__(
            name=name,
            description=description,
            status=status,
            importance=importance,
            blocking=blocking,
            should_display=should_display,
            parent_job_id=parent_job_id,
            order=order,
            event_metric=event_metric,
            tags=tags,
        )

        self.plotly_data = plotly_data

    def with_plotly_data(self: PlotlyMetric, plotly_data: str) -> PlotlyMetric:
        """Add a string json generated by the plotly to_json() function."""
        self.plotly_data = plotly_data
        return self

    def pack(self: PlotlyMetric) -> metrics_proto.Metric:
        msg = super().pack()
        msg.type = metrics_proto.MetricType.Value("PLOTLY_METRIC_TYPE")

        metric_values = msg.metric_values.plotly_metric_values

        if self.plotly_data is not None:
            struct_proto = Struct()
            Parse(self.plotly_data, struct_proto)
            metric_values.json.CopyFrom(struct_proto)

        return msg

    def recursively_pack_into(
        self: PlotlyMetric, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.job_level_metrics.metrics.extend([self.pack()])


@metric_dataclass
class ImageMetric(Metric["ImageMetric"]):
    image_data: Optional[ExternalFileMetricsData]

    def __init__(
        self: ImageMetric,
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        blocking: Optional[bool] = None,
        should_display: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
        image_data: Optional[ExternalFileMetricsData] = None,
    ):
        super().__init__(
            name=name,
            description=description,
            status=status,
            importance=importance,
            blocking=blocking,
            should_display=should_display,
            parent_job_id=parent_job_id,
            order=order,
            event_metric=event_metric,
            tags=tags,
        )

        self.image_data = image_data

    def with_image_data(
        self: ImageMetric, image_data: ExternalFileMetricsData
    ) -> ImageMetric:
        self.image_data = image_data
        return self

    def pack(self: ImageMetric) -> metrics_proto.Metric:
        msg = super().pack()
        msg.type = metrics_proto.MetricType.Value("IMAGE_METRIC_TYPE")

        metric_values = msg.metric_values.image_metric_values

        if self.image_data is not None:
            metric_values.image_data_id.id.CopyFrom(
                pack_uuid_to_proto(self.image_data.id)
            )

        return msg

    def recursively_pack_into(
        self: ImageMetric, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.job_level_metrics.metrics.extend([self.pack()])

        if self.image_data is not None:
            self.image_data.recursively_pack_into(metrics_output)


@metric_dataclass
class ImageListMetric(Metric["ImageListMetric"]):
    image_list_data: List[ExternalFileMetricsData]

    def __init__(
        self: ImageListMetric,
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        blocking: Optional[bool] = None,
        should_display: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
        image_list_data: Optional[List[ExternalFileMetricsData]] = None,
    ):
        super().__init__(
            name=name,
            description=description,
            status=status,
            importance=importance,
            blocking=blocking,
            should_display=should_display,
            parent_job_id=parent_job_id,
            order=order,
            event_metric=event_metric,
            tags=tags,
        )
        if image_list_data is None:
            self.image_list_data = []
        else:
            self.image_list_data = image_list_data

    def with_image_list_data(
        self: ImageListMetric, image_list_data: List[ExternalFileMetricsData]
    ) -> ImageListMetric:
        self.image_list_data = image_list_data
        return self

    def pack(self: ImageListMetric) -> metrics_proto.Metric:
        msg = super().pack()
        msg.type = metrics_proto.MetricType.Value("IMAGE_LIST_METRIC_TYPE")

        metric_values = msg.metric_values.image_list_metric_values

        if self.image_list_data is not None:
            for image_data in self.image_list_data:
                id_msg = metrics_proto.MetricsDataId()
                id_msg.id.CopyFrom(pack_uuid_to_proto(image_data.id))
                metric_values.image_data_ids.extend([id_msg])

        return msg

    def recursively_pack_into(
        self: ImageListMetric, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.job_level_metrics.metrics.extend([self.pack()])

        for data in self.image_list_data:
            data.recursively_pack_into(metrics_output)


@metric_dataclass
class TextMetric(Metric["TextMetric"]):
    text: Optional[str]

    def __init__(
        self: TextMetric,
        name: str,
        description: Optional[str] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        blocking: Optional[bool] = None,
        should_display: Optional[bool] = None,
        parent_job_id: Optional[uuid.UUID] = None,
        order: Optional[float] = None,
        event_metric: Optional[bool] = None,
        tags: Optional[List[Tag]] = None,
        text: Optional[str] = None,
    ):
        super().__init__(
            name=name,
            description=description,
            status=status,
            importance=importance,
            blocking=blocking,
            should_display=should_display,
            parent_job_id=parent_job_id,
            order=order,
            event_metric=event_metric,
            tags=tags,
        )

        self.text = text

    def with_text(self: TextMetric, text: str) -> TextMetric:
        """Add a string of text, which could be markdown."""
        self.text = text
        return self

    def pack(self: TextMetric) -> metrics_proto.Metric:
        msg = super().pack()
        msg.type = metrics_proto.MetricType.Value("TEXT_METRIC_TYPE")

        metric_values = msg.metric_values.text_metric_values
        metric_values.text = self.text

        return msg

    def recursively_pack_into(
        self: TextMetric, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.job_level_metrics.metrics.extend([self.pack()])


# -------------------
# Data representation
# -------------------


BaseMetricsDataT = TypeVar("BaseMetricsDataT", bound="BaseMetricsData")


@metric_dataclass
class BaseMetricsData(ABC, Generic[BaseMetricsDataT]):
    id: uuid.UUID
    name: str

    @abstractmethod
    def __init__(self: BaseMetricsDataT, name: str):
        assert name is not None
        self.id = uuid.uuid4()
        self.name = name

    def __eq__(self, __value: object) -> bool:
        if not isinstance(__value, type(self)):
            return False

        assert self.id is not None and __value.id is not None, (
            "Cannot compare values without valid ids"
        )

        return self.id == __value.id

    @abstractmethod
    def pack(self: BaseMetricsDataT) -> metrics_proto.MetricsData:
        raise NotImplementedError()

    @abstractmethod
    def recursively_pack_into(
        self: BaseMetricsDataT, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        output = self.pack()

        metrics_output.metrics_msg.metrics_data.extend([output])


MetricsDataT = TypeVar("MetricsDataT", bound="MetricsData")


@metric_dataclass
class MetricsData(BaseMetricsData[MetricsDataT]):
    unit: Optional[str] = None
    index_data: Optional[MetricsDataT]

    def __init__(
        self: MetricsDataT,
        name: str,
        unit: Optional[str] = None,
        index_data: Optional[MetricsDataT] = None,
    ):
        super().__init__(name)
        self.unit = unit
        self.index_data = index_data

    def with_unit(self: MetricsDataT, unit: str) -> MetricsDataT:
        self.unit = unit
        return self

    def with_index_data(self: MetricsDataT, index_data: MetricsDataT) -> MetricsDataT:
        self.index_data = index_data
        return self

    @abstractmethod
    def map(
        self: MetricsDataT,
        f: Callable[..., Any],
        applied_data_name: str,
        applied_unit: Optional[str] = None,
    ) -> MetricsDataT:
        raise NotImplementedError()

    @abstractmethod
    def group_by(
        self: MetricsDataT,
        grouping_series: MetricsDataT,
        grouped_data_name: Optional[str] = None,
        grouped_index_name: Optional[str] = None,
        override_grouped_index_data: Optional[GroupedMetricsData] = None,
    ) -> GroupedMetricsData:
        raise NotImplementedError()

    @abstractmethod
    def recursively_pack_into(
        self: MetricsDataT, metrics_output: ResimMetricsOutput
    ) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        output = self.pack()

        metrics_output.metrics_msg.metrics_data.extend([output])

        if self.index_data is not None:
            self.index_data.recursively_pack_into(metrics_output)


@metric_dataclass
class SeriesMetricsData(MetricsData["SeriesMetricsData"]):
    series: np.ndarray  # normally, dtype = 'object'

    def __init__(
        self: SeriesMetricsData,
        name: str,
        series: Optional[np.ndarray] = None,
        unit: Optional[str] = None,
        index_data: Optional[SeriesMetricsData] = None,
    ):
        super().__init__(name=name, unit=unit, index_data=index_data)
        self.series = series

    def with_series(self: SeriesMetricsData, series: np.ndarray) -> SeriesMetricsData:
        self.series = series
        return self

    def map(
        self: SeriesMetricsData,
        f: Callable[[np.ndarray, int], Any],
        applied_data_name: str,
        applied_unit: Optional[str] = None,
    ) -> SeriesMetricsData:
        new_series = np.empty_like(self.series)

        for i in range(len(self.series)):
            new_series[i] = f(self.series, i)

        return SeriesMetricsData(
            name=applied_data_name,
            series=new_series,
            unit=applied_unit,
            index_data=self.index_data,
        )

    def group_by(
        self: SeriesMetricsData,
        grouping_series: SeriesMetricsData,
        grouped_data_name: Optional[str] = None,
        grouped_index_name: Optional[str] = None,
        override_grouped_index_data: Optional[GroupedMetricsData] = None,
    ) -> GroupedMetricsData:
        assert self.series is not None

        assert grouping_series.series is not None
        assert len(self.series) == len(grouping_series.series)
        assert (
            self.index_data is None
            or grouping_series.index_data is None
            or self.index_data == grouping_series.index_data
            or self == grouping_series.index_data
            or self.index_data == grouping_series
        )

        grouped: Dict[Any, List[Any]] = defaultdict(list)
        for val, cat in zip(self.series, grouping_series.series):
            grouped[cat].append(val)

        # Convert the lists into numpy arrays for further processing
        for key, val in grouped.items():
            grouped[key] = np.array(val)

        grouped = dict(grouped)

        grouped_index_data = None
        if self.index_data is not None and override_grouped_index_data is None:
            index_series = self.index_data
            grouped_index: Dict[Any, List[Any]] = defaultdict(list)

            assert index_series.series is not None
            for val, cat in zip(index_series.series, grouping_series.series):
                grouped_index[cat].append(val)

            # Convert the lists into numpy arrays for further processing
            for key, val in grouped_index.items():
                grouped_index[key] = np.array(val)

            grouped_index = dict(grouped_index)

            grouped_index_data = GroupedMetricsData(
                (
                    grouped_index_name
                    if grouped_index_name is not None
                    else (f"{self.index_data.name}-grouped-by-{grouping_series.name}")
                ),
                category_to_series=grouped_index,
                unit=self.index_data.unit,
                index_data=None,
            )
        elif override_grouped_index_data is not None:
            grouped_index_data = override_grouped_index_data

        grouped_data = GroupedMetricsData(
            (
                grouped_data_name
                if grouped_data_name is not None
                else (f"{self.name}-grouped-by-{grouping_series.name}")
            ),
            category_to_series=grouped,
            unit=self.unit,
            index_data=grouped_index_data,
        )

        return grouped_data

    def pack(self: SeriesMetricsData) -> metrics_proto.MetricsData:
        msg = metrics_proto.MetricsData()
        msg.metrics_data_id.id.CopyFrom(pack_uuid_to_proto(self.id))
        msg.name = self.name

        if self.unit is not None:
            msg.unit = self.unit

        msg.is_per_category = False
        msg.is_indexed = self.index_data is not None

        if self.index_data is not None:
            msg.index_data_id.id.CopyFrom(pack_uuid_to_proto(self.index_data.id))
            msg.index_data_type, _ = pack_series_to_proto(
                self.index_data.series, self.index_data.index_data is not None
            )

        assert len(self.series) > 0, "Cannot pack an empty series."

        data_type, series = pack_series_to_proto(
            self.series, self.index_data is not None
        )

        msg.series.CopyFrom(series)
        msg.data_type = data_type

        return msg

    def recursively_pack_into(
        self: SeriesMetricsData, metrics_output: ResimMetricsOutput
    ) -> None:
        super().recursively_pack_into(metrics_output)


@metric_dataclass
class GroupedMetricsData(MetricsData["GroupedMetricsData"]):
    category_to_series: Dict[str, np.ndarray]  # normally, dtype='object'

    def __init__(
        self: GroupedMetricsData,
        name: str,
        category_to_series: Optional[Dict[str, np.ndarray]] = None,
        unit: Optional[str] = None,
        index_data: Optional[GroupedMetricsData] = None,
    ):
        super().__init__(name=name, unit=unit, index_data=index_data)
        if category_to_series is None:
            self.category_to_series = {}
        else:
            self.category_to_series = category_to_series

    def map(
        self: GroupedMetricsData,
        f: Callable[[np.ndarray, int, str], Any],
        applied_data_name: str,
        applied_unit: Optional[str] = None,
    ) -> GroupedMetricsData:
        new_category_to_series = {}
        for cat, series in self.category_to_series.items():
            new_category_to_series[cat] = np.empty_like(series)

            for i in range(len(series)):
                new_category_to_series[cat][i] = f(series, i, cat)

        return GroupedMetricsData(
            name=applied_data_name,
            category_to_series=new_category_to_series,
            unit=applied_unit,
            index_data=self.index_data,
        )

    def group_by(
        self: GroupedMetricsData,
        grouping_series: GroupedMetricsData,
        grouped_data_name: Optional[str] = None,
        grouped_index_name: Optional[str] = None,
        override_grouped_index_data: Optional[GroupedMetricsData] = None,
    ) -> GroupedMetricsData:
        raise NotImplementedError("Cannot group pre-grouped data")

    def with_category_to_series(
        self: GroupedMetricsData, category_to_series: Dict[str, np.ndarray]
    ) -> GroupedMetricsData:
        self.category_to_series = category_to_series
        return self

    def add_category(
        self: GroupedMetricsData, category: str, series: np.ndarray
    ) -> GroupedMetricsData:
        self.category_to_series[category] = series
        return self

    def pack(self: GroupedMetricsData) -> metrics_proto.MetricsData:
        msg = metrics_proto.MetricsData()
        msg.metrics_data_id.id.CopyFrom(pack_uuid_to_proto(self.id))
        msg.name = self.name

        assert len(self.category_to_series.keys()) > 0, (
            "Cannot pack grouped data with no categories."
        )

        categories = list(self.category_to_series.keys())
        if self.unit is not None:
            msg.unit = self.unit

        msg.is_per_category = True
        msg.category_names.extend(categories)
        msg.is_indexed = self.index_data is not None

        if self.index_data is not None:
            msg.index_data_id.id.CopyFrom(pack_uuid_to_proto(self.index_data.id))
            index_data_types = set()
            for cat in categories:
                series = self.index_data.category_to_series[cat]
                assert len(series) > 0, "Cannot pack an empty series."
                index_data_type, _ = pack_series_to_proto(
                    series, self.index_data.index_data is not None
                )
                index_data_types.add(index_data_type)
            err_str = f"Invalid number of index data types: {len(index_data_types)}"
            assert len(index_data_types) == 1, err_str
            msg.index_data_type = index_data_types.pop()

        data_types = set()
        for cat in categories:
            series = self.category_to_series[cat]
            assert len(series) > 0, "Cannot pack an empty series."

            data_type, series = pack_series_to_proto(
                series, self.index_data is not None
            )

            data_types.add(data_type)

            msg.series_per_category.category_to_series[cat].CopyFrom(series)

        assert len(data_types) == 1, f"Invalid number of data types: {len(data_types)}"
        msg.data_type = data_types.pop()

        return msg

    def recursively_pack_into(
        self: GroupedMetricsData, metrics_output: ResimMetricsOutput
    ) -> None:
        super().recursively_pack_into(metrics_output)


@metric_dataclass
class ExternalFileMetricsData(BaseMetricsData["ExternalFileMetricsData"]):
    filename: str

    def __init__(self: ExternalFileMetricsData, name: str, filename: str = ""):
        super().__init__(name=name)
        self.filename = filename

    def with_filename(
        self: ExternalFileMetricsData, filename: str
    ) -> ExternalFileMetricsData:
        self.filename = filename
        return self

    def pack(self: ExternalFileMetricsData) -> metrics_proto.MetricsData:
        msg = metrics_proto.MetricsData()
        msg.metrics_data_id.id.CopyFrom(pack_uuid_to_proto(self.id))
        msg.name = self.name
        msg.data_type = metrics_proto.EXTERNAL_FILE_DATA_TYPE
        msg.is_per_category = False

        assert len(self.filename) > 0, "Cannot pack an empty string."

        external_file = metrics_proto.ExternalFile()
        external_file.path = self.filename
        msg.external_file.CopyFrom(external_file)

        return msg

    def recursively_pack_into(
        self: ExternalFileMetricsData, metrics_output: ResimMetricsOutput
    ) -> None:
        super().recursively_pack_into(metrics_output)


# -------------------
# Event representation
# -------------------


@metric_dataclass
class Event:
    id: uuid.UUID
    name: str
    description: Optional[str]
    tags: Optional[list[str]]
    status: Optional[MetricStatus]
    importance: Optional[MetricImportance]
    timestamp: Optional[Timestamp]
    timestamp_type: Optional[TimestampType]
    metrics: Optional[List[Metric]]

    def __init__(
        self: Event,
        name: str,
        description: Optional[str] = None,
        tags: Optional[list[str]] = None,
        status: Optional[MetricStatus] = None,
        importance: Optional[MetricImportance] = None,
        timestamp: Optional[Timestamp] = None,
        timestamp_type: Optional[TimestampType] = None,
        metrics: Optional[List[Metric]] = None,
    ):
        assert name is not None
        self.id = uuid.uuid4()
        self.name = name
        self.description = description
        self.tags = tags
        self.status = status
        self.importance = importance
        self.timestamp = timestamp
        self.timestamp_type = timestamp_type
        self.metrics = metrics

    def __eq__(self: Event, __value: object) -> bool:
        if not isinstance(__value, type(self)):
            return False

        assert self.id is not None and __value.id is not None, (
            "Cannot compare values without valid ids"
        )

        return self.id == __value.id

    def with_description(self: Event, description: str) -> Event:
        self.description = description
        return self

    def with_status(self: Event, status: MetricStatus) -> Event:
        self.status = status
        return self

    def with_importance(self: Event, importance: MetricImportance) -> Event:
        self.importance = importance
        return self

    def with_tags(self: Event, tags: List[str]) -> Event:
        if isinstance(tags, str):
            raise ValueError(
                "`tags` must be a list and not a string. This is almost certainly a bug."
            )
        self.tags = tags
        return self

    def with_absolute_timestamp(self: Event, timestamp: Timestamp) -> Event:
        self.timestamp = timestamp
        self.timestamp_type = TimestampType.ABSOLUTE_TIMESTAMP
        return self

    def with_relative_timestamp(self: Event, timestamp: Timestamp) -> Event:
        self.timestamp = timestamp
        self.timestamp_type = TimestampType.RELATIVE_TIMESTAMP
        return self

    def with_metrics(self: Event, metrics: List[Metric]) -> Event:
        # ensure all event metrics have unique names
        assert len(metrics) == len(set(m.name for m in metrics)), (
            "Event metrics must have unique names."
        )
        self.metrics = metrics
        return self

    def pack(self: Event) -> metrics_proto.Event:
        msg = metrics_proto.Event()

        msg.event_id.id.CopyFrom(pack_uuid_to_proto(self.id))
        msg.name = self.name

        if self.description is not None:
            msg.description = self.description

        if self.status is not None:
            msg.status = self.status.value

        if self.importance is not None:
            msg.importance = self.importance.value

        if self.timestamp is not None:
            msg.timestamp.CopyFrom(self.timestamp.pack())

        if self.timestamp_type is not None:
            msg.timestamp_type = self.timestamp_type.value

        if self.tags is not None:
            msg.tags.extend(self.tags)

        if self.metrics is not None:
            msg.metrics.extend([pack_uuid_to_metric_id(m.id) for m in self.metrics])

        return msg

    def recursively_pack_into(self: Event, metrics_output: ResimMetricsOutput) -> None:
        if self.id in metrics_output.packed_ids:
            return
        metrics_output.packed_ids.add(self.id)

        metrics_output.metrics_msg.events.extend([self.pack()])
