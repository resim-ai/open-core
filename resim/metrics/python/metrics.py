from __future__ import annotations

from abc import ABC, abstractmethod
from collections import defaultdict
from dataclasses import dataclass
import uuid
from typing import Any, Callable, Dict, Generic, List, Optional, Set, TypeVar

import numpy as np

from resim.metrics.proto import metrics_pb2
from resim.metrics.proto.metrics_pb2 import MetricStatus, MetricImportance
from resim.utils.proto import uuid_pb2
from google.protobuf import timestamp_pb2

# ---------------
# Misc Data Types
# ---------------


@dataclass
class Timestamp:
    secs: int
    nanos: int

    def pack(self: Timestamp) -> timestamp_pb2.Timestamp:
        msg = timestamp_pb2.Timestamp()
        msg.seconds = self.secs
        msg.nanos = self.nanos
        return msg


@dataclass
class HistogramBucket:
    lower: float
    upper: float


@dataclass
class DoubleFailureDefinition:
    fails_above: Optional[float]
    fails_below: Optional[float]

    def pack(self: DoubleFailureDefinition) -> metrics_pb2.DoubleFailureDefinition:
        msg = metrics_pb2.DoubleFailureDefinition()
        if self.fails_above is not None:
            msg.fails_above = self.fails_above

        if self.fails_below is not None:
            msg.fails_below = self.fails_below

        return msg


def pack_uuid_to_proto(uuid_obj: uuid.UUID) -> uuid_pb2.UUID:
    uuid_msg = uuid_pb2.UUID()
    uuid_msg.data = str(uuid_obj)
    return uuid_msg


@dataclass(repr=True)
class ResimMetricsOutput:
    metrics_msg: metrics_pb2.MetricsData
    packed_ids: Set[uuid.UUID]

# ----------------------
# Overall metrics object
# ----------------------


@dataclass(init=False, repr=True, kw_only=True)
class ResimMetricsWriter:
    job_id: uuid.UUID
    metrics: Dict[uuid.UUID, Metric]
    metrics_data: Dict[uuid.UUID, MetricsData]

    names: Set[str]

    def __init__(self, job_id: uuid.UUID):
        self.job_id = job_id
        self.metrics = {}
        self.metrics_data = {}
        self.names = set()

    def add_metrics_data(self, data: MetricsData) -> MetricsData:
        assert data.name not in self.names
        self.names.add(data.name)
        self.metrics_data[data.id] = data
        return data

    def add_metric(self, metric: Metric) -> Metric:
        assert metric.name not in self.names
        self.names.add(metric.name)
        self.metrics[metric.id] = metric
        return metric

    def add_series_metrics_data(self, name: str) -> SeriesMetricsData:
        metrics_data = SeriesMetricsData(name=name)
        self.add_metrics_data(metrics_data)
        return metrics_data

    def add_grouped_metrics_data(self, name: str) -> GroupedMetricsData:
        metrics_data = GroupedMetricsData(name=name)
        self.add_metrics_data(metrics_data)
        return metrics_data

    def add_states_over_time_metric(self, name: str) -> StatesOverTimeMetric:
        metric = StatesOverTimeMetric(name=name)
        self.add_metric(metric)
        return metric

    def add_double_over_time_metric(self, name: str) -> DoubleOverTimeMetric:
        metric = DoubleOverTimeMetric(name=name)
        self.add_metric(metric)
        return metric

    def add_bar_chart_metric(self, name: str) -> BarChartMetric:
        metric = BarChartMetric(name=name)
        self.add_metric(metric)
        return metric

    def add_histogram_metric(self, name: str) -> HistogramMetric:
        metric = HistogramMetric(name=name)
        self.add_metric(metric)
        return metric

    def add_line_plot_metric(self, name: str) -> LinePlotMetric:
        metric = LinePlotMetric(name=name)
        self.add_metric(metric)
        return metric

    def add_scalar_metric(self, name: str) -> ScalarMetric:
        metric = ScalarMetric(name=name)
        self.add_metric(metric)
        return metric

    def add_double_summary_metric(self, name: str) -> DoubleSummaryMetric:
        metric = DoubleSummaryMetric(name=name)
        self.add_metric(metric)
        return metric

    def pack(self) -> ResimMetricsOutput:
        raise NotImplementedError()

# ---------------------
# Metric representation
# ---------------------


MetricType = TypeVar('MetricType', bound='Metric')


@dataclass(init=False, kw_only=True, repr=True)
class Metric(ABC, Generic[MetricType]):
    id: uuid.UUID
    name: str
    description: Optional[str]

    status: Optional[MetricStatus]
    importance: Optional[MetricImportance]

    should_display: Optional[bool]
    blocking: Optional[bool]

    @abstractmethod
    def __init__(self: MetricType,
                 name: str,
                 description: Optional[str] = None,
                 status: Optional[MetricStatus] = None,
                 importance: Optional[MetricImportance] = None,
                 should_display: Optional[bool] = None,
                 blocking: Optional[bool] = None):
        assert name is not None
        self.id = uuid.uuid4()
        self.name = name
        self.description = description
        self.status = status
        self.importance = importance
        self.should_display = should_display
        self.blocking = blocking

    def __eq__(self: MetricType, __value: object) -> bool:
        return type(self) == type(__value) and self.id == __value.id

    def with_description(self: MetricType, description: str) -> MetricType:
        self.description = description
        return self

    def with_status(self: MetricType, status: MetricStatus) -> MetricType:
        self.status = status
        return self

    def with_importance(self: MetricType, importance: MetricImportance) -> MetricType:
        self.importance = importance
        return self

    def with_should_display(self: MetricType, should_display: bool) -> MetricType:
        self.should_display = should_display
        return self

    def with_blocking(self: MetricType, blocking: bool) -> MetricType:
        self.blocking = blocking
        return self

    @abstractmethod
    def pack(self: MetricType) -> metrics_pb2.Metric:
        ...

    @abstractmethod
    def recursively_pack_into(self, job_metrics: ResimMetricsOutput) -> None:
        ...


@dataclass(init=False, kw_only=True, repr=True)
class ScalarMetric(Metric['ScalarMetric']):
    value: Optional[float]
    failure_definition: Optional[DoubleFailureDefinition]
    unit: Optional[str]

    def __init__(self: ScalarMetric,
                 name: str,
                 description: Optional[str] = None,
                 status: Optional[MetricStatus] = None,
                 importance: Optional[MetricImportance] = None,
                 should_display: Optional[bool] = None,
                 blocking: Optional[bool] = None,
                 value: Optional[float] = None,
                 failure_definition: Optional[DoubleFailureDefinition] = None,
                 unit: Optional[str] = None):
        super().__init__(name, description, status, importance, blocking, should_display)
        self.value = value
        self.failure_definition = failure_definition
        self.unit = unit

    def with_value(self: ScalarMetric, value: float) -> ScalarMetric:
        self.value = value
        return self

    def with_unit(self: ScalarMetric, unit: str) -> ScalarMetric:
        self.unit = unit
        return self

    def with_failure_definition(self: ScalarMetric, failure_definition: DoubleFailureDefinition) -> ScalarMetric:
        self.failure_definition = failure_definition
        return self

    def pack(self: ScalarMetric) -> metrics_pb2.Metric:
        raise NotImplementedError()

    def recursively_pack_into(self: ScalarMetric, job_metrics: ResimMetricsOutput) -> None:
        super().recursively_pack_into(job_metrics)
        raise NotImplementedError()


@dataclass(init=False, kw_only=True, repr=True)
class DoubleOverTimeMetric(Metric['DoubleOverTimeMetric']):
    doubles_over_time_data: Optional[List[MetricsData]]
    statuses_over_time_data: Optional[List[MetricsData]]

    failure_definition: Optional[DoubleFailureDefinition]

    start_time: Optional[Timestamp]
    end_time: Optional[Timestamp]

    y_axis_name: Optional[str]
    legend_series_names: Optional[List[Optional[str]]]

    def __init__(self: DoubleOverTimeMetric,
                 name: str,
                 description: Optional[str] = None,
                 status: Optional[MetricStatus] = None,
                 importance: Optional[MetricImportance] = None,
                 blocking: Optional[bool] = None,
                 should_display: Optional[bool] = None,
                 doubles_over_time_data: Optional[List[MetricsData]] = None,
                 statuses_over_time_data: Optional[List[MetricsData]] = None,
                 failure_definition: Optional[DoubleFailureDefinition] = None,
                 start_time: Optional[Timestamp] = None,
                 end_time: Optional[Timestamp] = None,
                 y_axis_name: Optional[str] = None,
                 legend_series_names: Optional[List[Optional[str]]] = None):
        super().__init__(name, description, status, importance, blocking, should_display)
        self.doubles_over_time_data = doubles_over_time_data
        self.statuses_over_time_data = statuses_over_time_data
        self.failure_definition = failure_definition
        self.start_time = start_time
        self.end_time = end_time
        self.y_axis_name = y_axis_name
        self.legend_series_names = legend_series_names

        if self.doubles_over_time_data is None:
            self.doubles_over_time_data = []

        if self.statuses_over_time_data is None:
            self.statuses_over_time_data = []

        if self.legend_series_names is None:
            self.legend_series_names = []

    def with_doubles_over_time_data(self: DoubleOverTimeMetric, doubles_over_time_data: List[MetricsData]) -> DoubleOverTimeMetric:
        self.doubles_over_time_data = doubles_over_time_data
        return self

    def with_statuses_over_time_data(self: DoubleOverTimeMetric, statuses_over_time_data: List[MetricsData]) -> DoubleOverTimeMetric:
        self.statuses_over_time_data = statuses_over_time_data
        return self

    def append_doubles_over_time_data(self: DoubleOverTimeMetric, double_over_time_data_element: MetricsData, legend_series_name: Optional[str] = None) -> DoubleOverTimeMetric:
        self.doubles_over_time_data.append(double_over_time_data_element)
        self.legend_series_names.append(legend_series_name)
        return self

    def with_failure_definition(self: DoubleOverTimeMetric, failure_definition: DoubleFailureDefinition) -> DoubleOverTimeMetric:
        self.failure_definition = failure_definition
        return self

    def with_start_time(self: DoubleOverTimeMetric, start_time: Timestamp) -> DoubleOverTimeMetric:
        self.start_time = start_time
        return self

    def with_end_time(self: DoubleOverTimeMetric, end_time: Timestamp) -> DoubleOverTimeMetric:
        self.end_time = end_time
        return self

    def with_y_axis_name(self: DoubleOverTimeMetric, y_axis_name: str) -> DoubleOverTimeMetric:
        self.y_axis_name = y_axis_name
        return self

    def with_legend_series_names(self: DoubleOverTimeMetric, legend_series_names: List[Optional[str]]) -> DoubleOverTimeMetric:
        self.legend_series_names = legend_series_names
        return self

    def pack(self: DoubleOverTimeMetric) -> metrics_pb2.Metric:
        raise NotImplementedError()

    def recursively_pack_into(self: DoubleOverTimeMetric, job_metrics: ResimMetricsOutput) -> None:
        super().recursively_pack_into(job_metrics)
        raise NotImplementedError()


@dataclass(init=False, kw_only=True, repr=True)
class StatesOverTimeMetric(Metric['StatesOverTimeMetric']):
    states_over_time_data: Optional[List[MetricsData]]
    statuses_over_time_data: Optional[List[MetricsData]]

    states_set: Optional[Set[str]]
    failure_states: Optional[Set[str]]

    legend_series_names: Optional[List[Optional[str]]]

    def __init__(self: StatesOverTimeMetric,
                 name: str,
                 description: Optional[str] = None,
                 status: Optional[MetricStatus] = None,
                 importance: Optional[MetricImportance] = None,
                 blocking: Optional[bool] = None,
                 should_display: Optional[bool] = None,
                 states_over_time_data: Optional[List[MetricsData]] = None,
                 statuses_over_time_data: Optional[List[MetricsData]] = None,
                 states_set: Optional[Set[str]] = None,
                 failures_states: Optional[Set[str]] = None,
                 legend_series_names: Optional[List[Optional[str]]] = None):
        super().__init__(name=name, description=description, status=status,
                         importance=importance, blocking=blocking, should_display=should_display)
        self.states_over_time_data = states_over_time_data

        self.statuses_over_time_data = statuses_over_time_data
        self.states_set = states_set
        self.failure_states = failures_states
        self.legend_series_names = legend_series_names

        if self.states_over_time_data is None:
            self.states_over_time_data = []

        if self.statuses_over_time_data is None:
            self.statuses_over_time_data = []

        if self.legend_series_names is None:
            self.legend_series_names = []

    def with_states_over_time_data(self: StatesOverTimeMetric, states_over_time_data: List[MetricsData]) -> StatesOverTimeMetric:
        self.states_over_time_data = states_over_time_data
        return self

    def with_states_over_time_series(self: StatesOverTimeMetric,
                                     states_over_time_series: Dict[str, np.ndarray],
                                     units: Optional[Dict[str, str]] = None,
                                     indices: Optional[Dict[str,
                                                            np.ndarray]] = None,
                                     legend_series_names: Optional[Dict[str, str]] = None) -> StatesOverTimeMetric:
        for name, series in states_over_time_series.items():
            unit, index_data = None, None
            if indices is not None and name in indices:
                index = indices[name]
                index_data = SeriesMetricsData(f'{name}_series_index', index)

            if units is not None and name in units:
                unit = units[name]

            data = SeriesMetricsData(name, series, unit, index_data)

            self.states_over_time_data.append(data)

            if legend_series_names is not None and name in legend_series_names:
                self.legend_series_names.append(legend_series_names[name])
            else:
                self.legend_series_names.append(None)

        return self

    def append_states_over_time_data(self: StatesOverTimeMetric, states_over_time_data_element: MetricsData, legend_series_name: Optional[str] = None) -> StatesOverTimeMetric:
        self.states_over_time_data.append(states_over_time_data_element)
        self.legend_series_names.append(legend_series_name)
        return self

    def with_statuses_over_time_data(self: StatesOverTimeMetric, statuses_over_time_data: List[MetricsData]) -> StatesOverTimeMetric:
        self.statuses_over_time_data = statuses_over_time_data
        return self

    def append_statuses_over_time_data(self: StatesOverTimeMetric, statuses_over_time_data_element: MetricsData) -> StatesOverTimeMetric:
        self.statuses_over_time_data.append(statuses_over_time_data_element)
        return self

    def with_states_set(self: StatesOverTimeMetric, states_set: Set[str]) -> StatesOverTimeMetric:
        self.states_set = states_set
        return self

    def with_failures_states(self: StatesOverTimeMetric, failures_states: Set[str]) -> StatesOverTimeMetric:
        self.failures_states = failures_states
        return self

    def with_legend_series_names(self: StatesOverTimeMetric, legend_series_names: List[Optional[str]]) -> StatesOverTimeMetric:
        self.legend_series_names = legend_series_names
        return self

    def pack(self: StatesOverTimeMetric) -> metrics_pb2.Metric:
        raise NotImplementedError()

    def recursively_pack_into(self: StatesOverTimeMetric, job_metrics: ResimMetricsOutput) -> None:
        raise NotImplementedError()


@dataclass(init=False, kw_only=True, repr=True)
class LinePlotMetric(Metric['LinePlotMetric']):
    x_doubles_data: Optional[List[MetricsData]]
    y_doubles_data: Optional[List[MetricsData]]
    statuses_data: Optional[List[MetricsData]]

    x_axis_name: Optional[str]
    y_axis_name: Optional[str]

    legend_series_names: Optional[List[Optional[str]]]

    def __init__(self: LinePlotMetric,
                 name: str,
                 description: Optional[str] = None,
                 status: Optional[MetricStatus] = None,
                 importance: Optional[MetricImportance] = None,
                 blocking: Optional[bool] = None,
                 should_display: Optional[bool] = None,
                 x_doubles_data: Optional[List[MetricsData]] = None,
                 y_doubles_data: Optional[List[MetricsData]] = None,
                 statuses_data: Optional[List[MetricsData]] = None,
                 x_axis_name: Optional[str] = None,
                 y_axis_name: Optional[str] = None,
                 legend_series_names: Optional[List[Optional[str]]] = None):
        super().__init__(name=name, description=description, status=status,
                         importance=importance, blocking=blocking, should_display=should_display)
        self.x_doubles_data = x_doubles_data
        self.y_doubles_data = y_doubles_data
        self.statuses_data = statuses_data

        self.x_axis_name = x_axis_name
        self.y_axis_name = y_axis_name
        self.legend_series_names = legend_series_names

        if self.x_doubles_data is None:
            self.x_doubles_data = []

        if self.y_doubles_data is None:
            self.y_doubles_data = []

        if self.statuses_data is None:
            self.statuses_data = []

        if self.legend_series_names is None:
            self.legend_series_names = []

    def add_series_data(self: LinePlotMetric, x_doubles_data: MetricsData, y_doubles_data: MetricsData, legend_series_name: Optional[str] = None) -> LinePlotMetric:
        self.x_doubles_data.append(x_doubles_data)
        self.y_doubles_data.append(y_doubles_data)
        self.legend_series_names.append(legend_series_name)

        return self

    def with_x_axis_name(self: LinePlotMetric, x_axis_name: str) -> LinePlotMetric:
        self.x_axis_name = x_axis_name
        return self

    def with_y_axis_name(self: LinePlotMetric, y_axis_name: str) -> LinePlotMetric:
        self.y_axis_name = y_axis_name
        return self

    def with_legend_series_names(self: LinePlotMetric, legend_series_names: List[Optional[str]]) -> LinePlotMetric:
        self.legend_series_names = legend_series_names
        return self

    def pack(self: LinePlotMetric) -> metrics_pb2.Metric:
        raise NotImplementedError()

    def recursively_pack_into(self: LinePlotMetric, job_metrics: ResimMetricsOutput) -> None:
        raise NotImplementedError()


@dataclass(init=False, kw_only=True, repr=True)
class BarChartMetric(Metric['BarChartMetric']):
    values_data: List[MetricsData]
    statuses_data: List[MetricsData]
    legend_series_names: List[Optional[str]]
    x_axis_name: str
    y_axis_name: str
    stack_bars: bool

    def __init__(self: BarChartMetric,
                 name: str,
                 description: Optional[str] = None,
                 status: Optional[MetricStatus] = None,
                 importance: Optional[MetricImportance] = None,
                 blocking: Optional[bool] = None,
                 should_display: Optional[bool] = None,
                 values_data: Optional[List[MetricsData]] = None,
                 statuses_data: Optional[List[MetricsData]] = None,
                 legend_series_names: Optional[List[Optional[str]]] = None,
                 x_axis_name: Optional[str] = None,
                 y_axis_name: Optional[str] = None,
                 stack_bars: Optional[bool] = None
                 ):
        super().__init__(name=name, description=description, status=status,
                         importance=importance, blocking=blocking, should_display=should_display)
        self.values_data = values_data
        self.statuses_data = statuses_data
        self.legend_series_names = legend_series_names
        self.x_axis_name = x_axis_name
        self.y_axis_name = y_axis_name
        self.stack_bars = stack_bars

        if self.values_data is None:
            self.values_data = []

        if self.statuses_data is None:
            self.statuses_data = []

        if self.legend_series_names is None:
            self.legend_series_names = []

    def append_values_data(self: BarChartMetric, values_data_element: MetricsData, legend_series_name: Optional[str] = None) -> BarChartMetric:
        self.values_data.append(values_data_element)
        self.legend_series_names.append(legend_series_name)
        return self

    def append_statuses_data(self: BarChartMetric, statuses_data_element: MetricsData) -> BarChartMetric:
        self.statuses_data.append(statuses_data_element)
        return self

    def with_x_axis_name(self: BarChartMetric, x_axis_name: str) -> BarChartMetric:
        self.x_axis_name = x_axis_name
        return self

    def with_y_axis_name(self: BarChartMetric, y_axis_name: str) -> BarChartMetric:
        self.y_axis_name = y_axis_name
        return self

    def with_legend_series_names(self: BarChartMetric, legend_series_names: List[Optional[str]]) -> BarChartMetric:
        self.legend_series_names = legend_series_names
        return self

    def with_stack_bars(self, stack_bars: bool) -> BarChartMetric:
        self.stack_bars = stack_bars
        return self

    def pack(self: BarChartMetric) -> metrics_pb2.Metric:
        raise NotImplementedError()

    def recursively_pack_into(self: BarChartMetric, job_metrics: ResimMetricsOutput) -> None:
        raise NotImplementedError()


@dataclass(init=False, kw_only=True, repr=True)
class HistogramMetric(Metric['HistogramMetric']):
    values_data: Optional[MetricsData]
    statuses_data: Optional[MetricsData]
    buckets: Optional[List[HistogramBucket]]
    lower_bound: Optional[float]
    upper_bound: Optional[float]

    def __init__(self: HistogramMetric,
                 name: str,
                 description: Optional[str] = None,
                 status: Optional[MetricStatus] = None,
                 importance: Optional[MetricImportance] = None,
                 blocking: Optional[bool] = None,
                 should_display: Optional[bool] = None,
                 values_data: Optional[MetricsData] = None,
                 statuses_data: Optional[MetricsData] = None,
                 buckets: Optional[List[HistogramBucket]] = None,
                 lower_bound: Optional[float] = None,
                 upper_bound: Optional[float] = None,
                 x_axis_name: Optional[str] = None
                 ):
        super().__init__(name=name, description=description, status=status,
                         importance=importance, blocking=blocking, should_display=should_display)

        self.values_data = values_data
        self.statuses_data = statuses_data
        self.buckets = buckets
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.x_axis_name = x_axis_name

    def with_values_data(self: HistogramMetric, values_data: MetricsData) -> HistogramMetric:
        self.values_data = values_data
        return self

    def with_statuses_data(self: HistogramMetric, statuses_data: MetricsData) -> HistogramMetric:
        self.statuses_data = statuses_data
        return self

    def with_buckets(self: HistogramMetric, buckets: List[HistogramBucket]) -> HistogramMetric:
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

    def pack(self: HistogramMetric) -> metrics_pb2.Metric:
        raise NotImplementedError()

    def recursively_pack_into(self: HistogramMetric, job_metrics: ResimMetricsOutput) -> None:
        raise NotImplementedError()


IndexType = TypeVar("IndexType", int, str, Timestamp, uuid.UUID, str)


@dataclass(init=False, kw_only=True, repr=True)
class DoubleSummaryMetric(Metric['DoubleSummaryMetric']):
    value_data: Optional[MetricsData]
    status_data: Optional[MetricsData]
    index: Optional[Generic[IndexType]]
    failure_definition: Optional[DoubleFailureDefinition]

    def __init__(self: DoubleSummaryMetric,
                 name: str,
                 description: Optional[str] = None,
                 status: Optional[MetricStatus] = None,
                 importance: Optional[MetricImportance] = None,
                 blocking: Optional[bool] = None,
                 should_display: Optional[bool] = None,
                 value_data: Optional[MetricsData] = None,
                 status_data: Optional[MetricsData] = None,
                 index: Optional[IndexType] = None,
                 failure_definition: Optional[DoubleFailureDefinition] = None,
                 ):
        super().__init__(name=name, description=description, status=status,
                         importance=importance, blocking=blocking, should_display=should_display)

        self.value_data = value_data
        self.status_data = status_data
        self.index = index
        self.failure_definition = failure_definition

    def with_value_data(self: DoubleSummaryMetric, value_data: MetricsData) -> DoubleSummaryMetric:
        self.value_data = value_data
        return self

    def with_status_data(self: DoubleSummaryMetric, status_data: MetricsData) -> DoubleSummaryMetric:
        self.status_data = status_data
        return self

    def with_index(self: DoubleSummaryMetric, index: IndexType) -> DoubleSummaryMetric:
        self.index = index
        return self

    def with_failure_definition(self: DoubleSummaryMetric, failure_definition: DoubleFailureDefinition) -> DoubleSummaryMetric:
        self.failure_definition = failure_definition
        return self

    def pack(self: DoubleSummaryMetric) -> metrics_pb2.Metric:
        raise NotImplementedError()

    def recursively_pack_into(self: DoubleSummaryMetric, job_metrics: ResimMetricsOutput) -> None:
        raise NotImplementedError()

# -------------------
# Data representation
# -------------------


MetricsDataType = TypeVar('MetricsDataType', bound='MetricsData')


@dataclass(init=False, kw_only=True, repr=True)
class MetricsData(ABC, Generic[MetricsDataType]):
    id: uuid.UUID
    name: str
    unit: Optional[str]
    index_data: Optional[MetricsData]

    @abstractmethod
    def __init__(self: MetricsDataType,
                 name: str,
                 unit: Optional[str] = None,
                 index_data: Optional[MetricsData] = None):
        assert name is not None
        self.id = uuid.uuid4()
        self.name = name
        self.unit = unit
        self.index_data = index_data

    def __eq__(self, __value: object) -> bool:
        return type(self) == type(__value) and self.id == __value.id

    def with_unit(self, unit: str) -> MetricsData[MetricsDataType]:
        self.unit = unit
        return self

    def with_index_data(self, index_data: MetricsData) -> MetricsData[MetricsDataType]:
        self.index_data = index_data
        return self

    @abstractmethod
    def pack(self: MetricsDataType) -> metrics_pb2.MetricsData:
        ...

    @abstractmethod
    def recursively_pack_into(self: MetricsDataType, job_metrics: ResimMetricsOutput) -> None:
        ...


@dataclass(init=False, kw_only=True, repr=True)
class SeriesMetricsData(MetricsData['SeriesMetricsData']):
    series: np.ndarray  # normally, dtype = 'object'

    def __init__(self: SeriesMetricsData,
                 name: str,
                 series: Optional[np.ndarray] = None,
                 unit: Optional[str] = None,
                 index_data: Optional[MetricsData] = None):
        super().__init__(name, unit, index_data)
        self.series = series

    def with_series(self: SeriesMetricsData, series: np.ndarray) -> SeriesMetricsData:
        self.series = series
        return self

    def map(self: SeriesMetricsData,
            f: Callable[[np.ndarray, int], Any],
            applied_data_name: str,
            applied_unit: Optional[str] = None) -> SeriesMetricsData:
        new_series = np.empty_like(self.series)

        for i in range(len(self.series)):
            new_series[i] = f(self.series, i)

        return SeriesMetricsData(
            name=applied_data_name,
            series=new_series,
            unit=applied_unit,
            index_data=self.index_data
        )

    def group_by(self: SeriesMetricsData,
                 grouping_series: SeriesMetricsData,
                 grouped_data_name: Optional[str] = None,
                 grouped_index_name: Optional[str] = None) -> GroupedMetricsData:
        assert self.series is not None
        assert grouping_series.series is not None
        assert len(self.series) == len(grouping_series.series)
        assert grouping_series.index_data is None or grouping_series.index_data == self.index_data

        grouped = defaultdict(list)
        for val, cat in zip(self.series, grouping_series.series):
            grouped[cat].append(val)

        # Convert the lists into numpy arrays for further processing
        for key, val in grouped.items():
            grouped[key] = np.array(val)

        grouped = dict(grouped)

        grouped_index_data = None
        if self.index_data is not None:
            grouped_index = defaultdict(list)
            assert self.index_data.series is not None
            for val, cat in zip(self.index_data.series, grouping_series.series):
                grouped_index[cat].append(val)

            # Convert the lists into numpy arrays for further processing
            for key, val in grouped_index.items():
                grouped_index[key] = np.array(val)

            grouped_index = dict(grouped_index)

            grouped_index_data = GroupedMetricsData(
                grouped_index_name if grouped_index_name is not None else (
                    f'{self.index_data.name}-grouped-by-{grouping_series.name}'
                ),
                category_to_series=grouped_index,
                unit=self.unit,
                index_data=None
            )

        grouped_data = GroupedMetricsData(
            grouped_data_name if grouped_data_name is not None else (
                f'{self.name}-grouped-by-{grouping_series.name}'),
            category_to_series=grouped,
            unit=self.unit,
            index_data=grouped_index_data
        )

        return grouped_data

    def pack(self: SeriesMetricsData) -> metrics_pb2.MetricsData:
        raise NotImplementedError()

    def recursively_pack_into(self: SeriesMetricsData, job_metrics: ResimMetricsOutput) -> None:
        raise NotImplementedError()


@dataclass(init=False, kw_only=True, repr=True)
class GroupedMetricsData(MetricsData['GroupedMetricsData']):
    category_to_series: Dict[str, np.ndarray]  # normally, dtype='object'

    def __init__(self: GroupedMetricsData,
                 name: str,
                 category_to_series: Optional[Dict[str, np.ndarray]] = None,
                 unit: Optional[str] = None,
                 index_data: Optional[MetricsData] = None):
        super().__init__(name, unit, index_data)
        self.category_to_series = category_to_series

    def map(self: GroupedMetricsData,
            f: Callable[[np.ndarray, int, str], Any],
            applied_data_name: str,
            applied_unit: Optional[str] = None) -> GroupedMetricsData:
        new_category_to_series = {}
        for cat, series in self.category_to_series.items():
            new_category_to_series[cat] = np.empty_like(series)

            for i in range(len(series)):
                new_category_to_series[cat][i] = f(series, i, cat)

        return GroupedMetricsData(
            name=applied_data_name,
            category_to_series=new_category_to_series,
            unit=applied_unit,
            index_data=self.index_data
        )

    def with_category_to_series(self: GroupedMetricsData,
                                category_to_series: Dict[str, np.ndarray]) -> GroupedMetricsData:
        self.category_to_series = category_to_series
        return self

    def pack(self: GroupedMetricsData) -> metrics_pb2.MetricsData:
        raise NotImplementedError()

    def recursively_pack_into(self: GroupedMetricsData, job_metrics: ResimMetricsOutput) -> None:
        raise NotImplementedError()
