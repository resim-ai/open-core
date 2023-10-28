from __future__ import annotations
import random

from abc import ABC, abstractmethod
from collections import defaultdict
from dataclasses import dataclass
import uuid
from typing import Any, Callable, Dict, List, Optional, Set

import numpy as np

from resim.metrics.proto import metrics_pb2
from resim.metrics.proto.metrics_pb2 import MetricStatus, MetricImportance
from resim.utils.proto import uuid_pb2
from google.protobuf import timestamp_pb2

# ----
# Misc
# ----


@dataclass
class Timestamp:
    secs: int
    nanos: int

    def pack(self) -> timestamp_pb2.Timestamp():
        msg = timestamp_pb2.Timestamp()
        msg.seconds = self.secs
        msg.nanos = self.nanos
        return msg


@dataclass
class DoubleFailureDefinition:
    fails_above: float
    fails_below: float

# ----------------------
# Overall metrics object
# ----------------------


class ResimMetrics:
    def __init__(self, job_id: UUID):
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


class Metric(ABC):
    @abstractmethod
    def __init__(self,
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

    def __eq__(self, __value: object) -> bool:
        return type(self) == type(__value) and self.id == __value.id

    def with_description(self, description: str) -> Metric:
        self.description = description
        return self

    def with_status(self, status: MetricStatus) -> Metric:
        self.status = status
        return self

    def with_importance(self, importance: MetricImportance) -> Metric:
        self.importance = importance
        return self

    def with_should_display(self, should_display: bool) -> Metric:
        self.should_display = should_display
        return self

    def with_blocking(self, blocking: bool) -> Metric:
        self.blocking = blocking
        return self


class ScalarMetric(Metric):
    def __init__(self,
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

    def with_value(self, value: float) -> ScalarMetric:
        self.value = value
        return self

    def with_unit(self, unit: str) -> ScalarMetric:
        self.unit = unit
        return self

    def with_failure_definition(self, failure_definition: DoubleFailureDefinition) -> ScalarMetric:
        self.failure_definition = failure_definition
        return self

    def pack(self) -> metrics_pb2.MetricsData:
        raise NotImplementedError()


class DoubleOverTimeMetric(Metric):
    def __init__(self,
                 name: str,
                 description: Optional[str] = None,
                 status: Optional[MetricStatus] = None,
                 importance: Optional[MetricImportance] = None,
                 blocking: Optional[bool] = None,
                 should_display: Optional[bool] = None,
                 doubles_over_time_data: Optional[MetricsData] = None,
                 statuses_over_time_data: Optional[MetricsData] = None,
                 failure_definition: Optional[DoubleFailureDefinition] = None,
                 start_time: Optional[Timestamp] = None,
                 end_time: Optional[Timestamp] = None,
                 y_axis_name: Optional[str] = None,
                 legend_series_name: Optional[List[str]] = None):
        super().__init__(name, description, status, importance, blocking, should_display)
        raise NotImplementedError()

    def with_doubles_over_time_data(self, doubles_over_time_data: MetricsData) -> MetricsData:
        self.doubles_over_time_data = doubles_over_time_data
        return self

    def pack(self) -> metrics_pb2.MetricsData:
        raise NotImplementedError()

    # TODO(tknowles): Lots of missing methods here!


class StatesOverTimeMetric(Metric):
    def __init__(self,
                 name: str,
                 description: Optional[str] = None,
                 status: Optional[MetricStatus] = None,
                 importance: Optional[MetricImportance] = None,
                 blocking: Optional[bool] = None,
                 should_display: Optional[bool] = None,
                 states_over_time_data: List[MetricsData] = [],
                 statuses_over_time_data: List[MetricsData] = [],
                 states_set: Optional[Set[str]] = None,
                 failures_states: Optional[Set[str]] = None,
                 legend_series_names: Optional[List[str]] = None):

        super().__init__(name=name, description=description, status=status,
                         importance=importance, blocking=blocking, should_display=should_display)
        self.states_over_time_data = states_over_time_data
        if (name == "Test example"):
            print('Test example', statuses_over_time_data)

        self.statuses_over_time_data = statuses_over_time_data
        self.states_set = states_set
        self.failure_states = failures_states
        self.legend_series_names = legend_series_names

    def with_states_over_time_data(self, states_over_time_data: List[MetricsData]) -> StatesOverTimeMetric:
        self.states_over_time_data = states_over_time_data
        return self

    def with_states_over_time_series(self,
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

    def append_states_over_time_data(self, states_over_time_data_element: MetricsData) -> StatesOverTimeMetric:
        self.states_over_time_data.append(states_over_time_data_element)
        return self

    def with_statuses_over_time_data(self, statuses_over_time_data: List[MetricsData]) -> StatesOverTimeMetric:
        self.statuses_over_time_data = statuses_over_time_data
        return self

    def append_statuses_over_time_data(self, statuses_over_time_data_element: MetricsData) -> StatesOverTimeMetric:
        self.statuses_over_time_data.append(statuses_over_time_data_element)
        return self

    def with_states_set(self, states_set: Set[str]) -> StatesOverTimeMetric:
        self.states_set = states_set
        return self

    def with_failures_states(self, failures_states: Set[str]) -> StatesOverTimeMetric:
        self.failures_states = failures_states
        return self

    def with_legend_series_names(self, legend_series_names: List[str]) -> StatesOverTimeMetric:
        self.legend_series_names = legend_series_names
        return self

    def pack(self) -> metrics_pb2.MetricsData:
        raise NotImplementedError()


# Data representation

class MetricsData(ABC):
    @abstractmethod
    def __init__(self,
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


class SeriesMetricsData(MetricsData):
    def __init__(self,
                 name: str,
                 series: Optional[np.ndarray] = None,
                 unit: Optional[str] = None,
                 index_data: Optional[MetricsData] = None):
        super().__init__(name, unit, index_data)
        self.series = series

    def with_series(self, series: np.ndarray) -> SeriesMetricsData:
        self.series = series
        return self

    def map(self, f: Callable[[np.ndarray, int], Any], applied_data_name: str, applied_unit: Optional[str] = None) -> SeriesMetricsData:
        new_series = np.empty_like(self.series)

        for i in range(len(self.series)):
            new_series[i] = f(self.series, i)

        return SeriesMetricsData(
            name=applied_data_name,
            series=new_series,
            unit=applied_unit,
            index_data=self.index_data
        )

    def group_by(self, grouping_series: SeriesMetricsData,
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

    def pack(self) -> metrics_pb2.MetricsData:
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


class GroupedMetricsData(MetricsData):
    category_to_series: Dict[str, np.ndarray]  # normally, dtype='object'

    def __init__(self,
                 name: str,
                 category_to_series: Optional[Dict[str, np.ndarray]] = None,
                 unit: Optional[str] = None,
                 index_data: Optional[MetricsData] = None):
        super().__init__(name, unit, index_data)
        self.category_to_series = category_to_series

    def map(self, f: Callable[[np.ndarray, int, str], Any], applied_data_name: str, applied_unit: Optional[str] = None) -> SeriesMetricsData:
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

    def with_category_to_series(self, category_to_series: Dict[str, np.ndarray]) -> GroupedMetricsData:
        self.category_to_series = category_to_series
        return self

    def pack(self) -> metrics_pb2.MetricsData:
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
