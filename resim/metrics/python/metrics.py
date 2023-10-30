from __future__ import annotations
import random

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

    def pack(self) -> metrics_pb2.JobMetrics:
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
    def recursively_pack_into(self, job_metrics: metrics_pb2.JobMetrics):
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

    def pack(self: ScalarMetric) -> metrics_pb2.MetricsData:
        raise NotImplementedError()

    def recursively_pack_into(self: ScalarMetric, job_metrics: metrics_pb2.JobMetrics):
        return super().recursively_pack_into(job_metrics)
        raise NotImplementedError()


@dataclass(init=False, kw_only=True, repr=True)
class DoubleOverTimeMetric(Metric['DoubleOverTimeMetric']):
    doubles_over_time: Optional[List[MetricsData]]
    statuses_over_time: Optional[List[MetricsData]]

    failure_definition: Optional[DoubleFailureDefinition]

    start_time: Optional[Timestamp]
    end_time: Optional[Timestamp]

    y_axis_name: Optional[str]
    legend_series_names: Optional[List[str]]

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
                 legend_series_names: Optional[List[str]] = None):
        super().__init__(name, description, status, importance, blocking, should_display)
        self.doubles_over_time_data = doubles_over_time_data
        self.statuses_over_time_data = statuses_over_time_data
        self.failure_definition = failure_definition
        self.start_time = start_time
        self.end_time = end_time
        self.y_axis_name = y_axis_name
        self.legend_series_names = legend_series_names

    def with_doubles_over_time_data(self: DoubleOverTimeMetric, doubles_over_time_data: MetricsData) -> DoubleOverTimeMetric:
        self.doubles_over_time_data = doubles_over_time_data
        return self

    def with_statuses_over_time_data(self: DoubleOverTimeMetric, statuses_over_time_data: List[MetricsData]) -> DoubleOverTimeMetric:
        self.statuses_over_time_data = statuses_over_time_data
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

    def with_legend_series_names(self: DoubleOverTimeMetric, legend_series_names: List[str]) -> DoubleOverTimeMetric:
        self.legend_series_names = legend_series_names
        return self

    def pack(self: DoubleOverTimeMetric) -> metrics_pb2.Metric:
        packed_metric = super().pack()
        raise NotImplementedError()

    def recursively_pack_into(self: DoubleOverTimeMetric, job_metrics: metrics_pb2.JobMetrics):
        super().recursively_pack_into(job_metrics)


@dataclass(init=False, kw_only=True, repr=True)
class StatesOverTimeMetric(Metric['StatesOverTimeMetric']):
    states_over_time_data: Optional[MetricsData]
    statuses_over_time_data: Optional[MetricsData]

    states_set: Optional[Set[str]]
    failure_states: Optional[Set[str]]

    legend_series_names: Optional[List[str]]

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
                 legend_series_names: Optional[List[str]] = None):
        super().__init__(name=name, description=description, status=status,
                         importance=importance, blocking=blocking, should_display=should_display)
        self.states_over_time_data = states_over_time_data

        self.statuses_over_time_data = statuses_over_time_data
        self.states_set = states_set
        self.failure_states = failures_states
        self.legend_series_names = legend_series_names

    def with_states_over_time_data(self: StatesOverTimeMetric, states_over_time_data: List[MetricsData]) -> StatesOverTimeMetric:
        self.states_over_time_data = states_over_time_data
        return self

    def with_states_over_time_series(self: StatesOverTimeMetric,
                                     states_over_time_series: Dict[str, np.ndarray],
                                     units: Dict[str, str] = None,
                                     indices: Dict[str, np.ndarray] = None) -> StatesOverTimeMetric:
        # Major TODO before pack, if we're keeping this method: Make sure these series are actually added to metrics object
        for name, series in states_over_time_series.items():
            unit, index_data = None, None
            if indices is not None and name in indices:
                index = indices[name]
                index_data = SeriesMetricsData(f'{name}_series_index', index)

            if units is not None and name in units:
                unit = units[name]

            data = SeriesMetricsData(name, series, unit, index_data)

            self.states_over_time_data.append(data)

        return self

    def append_states_over_time_data(self: StatesOverTimeMetric, states_over_time_data_element: MetricsData) -> StatesOverTimeMetric:
        if self.states_over_time_data is None:
            self.states_over_time_data = []
        self.states_over_time_data.append(states_over_time_data_element)
        return self

    def with_statuses_over_time_data(self: StatesOverTimeMetric, statuses_over_time_data: List[MetricsData]) -> StatesOverTimeMetric:
        self.statuses_over_time_data = statuses_over_time_data
        return self

    def append_statuses_over_time_data(self: StatesOverTimeMetric, statuses_over_time_data_element: MetricsData) -> StatesOverTimeMetric:
        if self.statuses_over_time_data is None:
            self.statuses_over_time_data = []
        self.statuses_over_time_data.append(statuses_over_time_data_element)
        return self

    def with_states_set(self: StatesOverTimeMetric, states_set: Set[str]) -> StatesOverTimeMetric:
        self.states_set = states_set
        return self

    def with_failures_states(self: StatesOverTimeMetric, failures_states: Set[str]) -> StatesOverTimeMetric:
        self.failures_states = failures_states
        return self

    def with_legend_series_names(self: StatesOverTimeMetric, legend_series_names: List[str]) -> StatesOverTimeMetric:
        self.legend_series_names = legend_series_names
        return self

    def pack(self: StatesOverTimeMetric) -> metrics_pb2.MetricsData:
        raise NotImplementedError()

    def recursively_pack_into(self: StatesOverTimeMetric, job_metrics: metrics_pb2.JobMetrics):
        raise NotImplementedError()

# Data representation


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

    def with_unit(self, unit: str) -> MetricsData:
        self.unit = unit
        return self

    def with_index_data(self, index_data: MetricsData) -> MetricsData:
        self.index_data = index_data
        return self

    @abstractmethod
    def pack(self: MetricsDataType) -> metrics_pb2.MetricsData:
        ...

    @abstractmethod
    def recursively_pack_into(self: MetricsDataType, job_metrics: metrics_pb2.JobMetrics):
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
        assert grouping_series.index_data == None or grouping_series.index_data == self.index_data

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

    def recursively_pack_into(self: SeriesMetricsData, job_metrics: metrics_pb2.JobMetrics):
        raise NotImplementedError()


def pack_uuid_to_proto(uuid: uuid.UUID) -> uuid_pb2.UUID:
    uuid_msg = uuid_pb2.UUID()
    uuid.data = str(uuid)
    return uuid_msg


def pack_series_to_proto(series: np.ndarray, indexed: bool):
    data_type, series_msg = metrics_pb2.MetricsDataType.Value(
        'NO_DATA_TYPE'), metrics_pb2.Series()

    if len(series) == 0:
        data_type = metrics_pb2.MetricsDataType.Value('NO_DATA_TYPE')
    elif isinstance(series[0], float):
        if not indexed:
            data_type = metrics_pb2.MetricsDataType.Value(
                'DOUBLE_SERIES_DATA_TYPE')
        else:
            data_type = metrics_pb2.MetricsDataType.Value(
                'INDEXED_DOUBLE_SERIES_DATA_TYPE')
        series_msg.doubles.series.extend(list(series))
    elif isinstance(series[0], Timestamp):
        if not indexed:
            data_type = metrics_pb2.MetricsDataType.Value(
                'TIMESTAMP_SERIES_DATA_TYPE')
        else:
            data_type = metrics_pb2.MetricsDataType.Value(
                'INDEXED_TIMESTAMP_SERIES_DATA_TYPE')
        series_msg.timestamps.series.extend([t.pack() for t in series])
    elif isinstance(series[0], uuid.UUID):
        if not indexed:
            data_type = metrics_pb2.MetricsDataType.Value(
                'UUID_SERIES_DATA_TYPE')
        else:
            data_type = metrics_pb2.MetricsDataType.Value(
                'INDEXED_UUID_SERIES_DATA_TYPE')
        series_msg.uuids.series.extend([pack_uuid_to_proto(i) for i in series])
    elif isinstance(series[0], str):
        if not indexed:
            data_type = metrics_pb2.MetricsDataType.Value(
                'STRING_SERIES_DATA_TYPE')
        else:
            data_type = metrics_pb2.MetricsDataType.Value(
                'INDEXED_STRING_SERIES_DATA_TYPE')
        series_msg.strings.series.extend(list(series))
    elif isinstance(series[0], MetricStatus):
        if not indexed:
            data_type = metrics_pb2.MetricsDataType.Value(
                'METRIC_STATUS_SERIES_DATA_TYPE')
        else:
            data_type = metrics_pb2.MetricsDataType.Value(
                'INDEXED_METRIC_STATUS_SERIES_DATA_TYPE')
        series_msg.statuses.series.extend(list(series))
    else:
        raise ValueError("Invalid data type packed.")

    return data_type, series_msg


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
        msg = metrics_pb2.MetricsData()
        msg.metrics_data_id.id.data = str(self.id)

        assert len(self.category_to_series.keys()
                   ) > 0, "Cannot pack grouped data with no categories."
        categories = list(self.category_to_series.keys())
        example_category = categories[0]
        example_series = self.category_to_series[example_category]
        assert len(example_series) > 0, "Cannot pack an empty series."

        if self.unit is not None:
            msg.unit = self.unit

        msg.is_per_category = True
        msg.category_names.extend(list(self.category_to_series.keys()))

        msg.is_indexed = (self.index_data is not None)

        if self.index_data is not None:
            msg.index_data_id.id.data = str(self.id)

        for cat, series in self.category_to_series.items():
            assert len(series) > 0, "Cannot pack an empty series."

            data_type, series = pack_series_to_proto(
                series, self.index_data is not None)

            msg.series_per_category.category_to_series[cat].CopyFrom(series)
            assert msg.data_type == 0 or msg.data_type == data_type

    def pack(self: GroupedMetricsData) -> metrics_pb2.MetricsData:
        raise NotImplementedError()

    def recursively_pack_into(self: GroupedMetricsData, job_metrics: metrics_pb2.JobMetrics):
        raise NotImplementedError()
