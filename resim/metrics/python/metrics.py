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

# Misc


@dataclass
class Timestamp:
    secs: int
    nanos: int


@dataclass
class DoubleFailureDefinition:
    fails_above: float
    fails_below: float

# Overall metrics object


class ResimMetrics:
    def __init__(self):
        raise NotImplementedError()

    def add_metrics_data(self, data: MetricsData) -> MetricsData:
        # Only adds if type is valid for job metrics data - i.e. can be packed.
        # Also adds index
        raise NotImplementedError()

    def add_metric(self, metric: Metric) -> Metric:
        raise NotImplementedError()

    def pack(self) -> metrics_pb2.JobMetrics:
        raise NotImplementedError()

# Metrics representation


@dataclass(init=False, repr=True)
class Metric(ABC):
    id: uuid.UUID
    name: str
    description: Optional[str]

    status: Optional[MetricStatus]
    importance: Optional[MetricImportance]

    should_display: Optional[bool]
    blocking: Optional[bool]

    @abstractmethod
    def __init__(self,
                 name: str,
                 description: Optional[str],
                 status: Optional[MetricStatus],
                 importance: Optional[MetricImportance],
                 should_display: Optional[bool],
                 blocking: Optional[bool]):
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


@dataclass(init=False, repr=True)
class ScalarMetric(Metric):
    value: float

    failure_definition: Optional[DoubleFailureDefinition]
    unit: Optional[str]

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
        raise NotImplementedError()

    def pack(self) -> metrics_pb2.MetricsData:
        raise NotImplementedError()


@dataclass(init=False, repr=True)
class DoubleOverTimeMetric(Metric):
    doubles_over_time: Optional[MetricsData]
    statuses_over_time: Optional[MetricsData]

    failure_definition: Optional[DoubleFailureDefinition]

    start_time: Optional[Timestamp]
    end_time: Optional[Timestamp]

    y_axis_name: Optional[str]
    legend_series_names: Optional[List[str]]

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


@dataclass(init=False, repr=True)
class StatesOverTimeMetric(Metric):
    states_over_time_data: Optional[MetricsData]
    statuses_over_time_data: Optional[MetricsData]

    states_set: Optional[Set[str]]
    failure_states: Optional[Set[str]]

    legend_series_names: Optional[List[str]]

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
        super().__init__(name, description, status, importance, blocking, should_display)
        self.states_over_time_data = states_over_time_data
        self.statuses_over_time_data = statuses_over_time_data
        self.states_set = states_set
        self.failure_states = failures_states
        self.legend_series_names = legend_series_names

    def with_states_over_time_data(self, states_over_time_data: List[MetricsData]) -> StatesOverTimeMetric:
        self.states_over_time_data = states_over_time_data
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

@dataclass(init=False, repr=True)
class MetricsData:
    id: uuid.UUID
    name: str
    unit: Optional[str]
    index_data: Optional[MetricsData]

    @abstractmethod
    def __init__(self,
                 name: str,
                 unit: Optional[str],
                 index_data: Optional[MetricsData]):
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


@dataclass(init=False, repr=True)
class SeriesMetricsData(MetricsData):
    series: np.ndarray  # normally, dtype='object'

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

    def group_by(self, grouping_series: SeriesMetricsData, grouped_data_name: Optional[str] = None) -> GroupedMetricsData:
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

        grouped_index = None
        if self.index_data is not None:
            grouped_index = defaultdict(list)
            for val, cat in zip(self.index_data.series, grouping_series.series):
                grouped_index[cat].append(val)

            # Convert the lists into numpy arrays for further processing
            for key, val in grouped_index.items():
                grouped_index[key] = np.array(val)

            grouped_index = dict(grouped_index)

        grouped_data = GroupedMetricsData(
            grouped_data_name if grouped_data_name is not None else (
                f'{self.name}-grouped-by-{grouping_series.name}'),
            category_to_series=grouped,
            unit=self.unit,
            index_data=grouped_index
        )

        return grouped_data

    def pack(self) -> metrics_pb2.MetricsData:
        raise NotImplementedError()


@dataclass(init=False, repr=True)
class GroupedMetricsData(MetricsData):
    category_to_series: Dict[str, np.ndarray]  # normally, dtype='object'

    def __init__(self,
                 name: str,
                 category_to_series: Optional[Dict[str, np.ndarray]],
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
        msg.name = self.name

        if self.unit is not None:
            msg.unit = self.unit

        if self.index_data is not None:
            msg.



random.seed(194842)

# Fake data, assume this is read in
NUM_ACTORS = 10
EXAMPLE_ID_SET = [uuid.uuid4() for i in range(NUM_ACTORS)]
EXAMPLE_COUNTS = [10 * (i + 1) for i in range(NUM_ACTORS)]
EXAMPLE_STATES_SET = ['CAR', 'TRUCK', 'BIKE',
                      'PEDESTRIAN', 'DEBRIS', 'UNKNOWN']
EXAMPLE_DETECTIONS_SET = ['CAR_VEHICLE', 'TRUCK_VEHICLE',
                          'BIKE_VEHICLE', 'PEDESTRIAN_VEHICLE', 'DEBRIS_VEHICLE', 'UNKNOWN']
EXAMPLE_FAILURE_STATES = ['UNKNOWN']
EXAMPLE_LEGEND_SERIES_NAMES = ['Labels', 'Detections']

EXAMPLE_IDS = np.array([EXAMPLE_ID_SET[i] for i in range(
    NUM_ACTORS) for _ in range(EXAMPLE_COUNTS[i])])
EXAMPLE_TIMESTAMPS = np.array(
    [Timestamp(secs=i + 1, nanos=0) for i, _ in enumerate(EXAMPLE_IDS)])
EXAMPLE_LABELS = np.array([random.choice(EXAMPLE_STATES_SET)
                          for _ in EXAMPLE_IDS])
EXAMPLE_DETECTIONS = np.array(
    [random.choice(EXAMPLE_DETECTIONS_SET) for _ in EXAMPLE_IDS])

# Actual data code
timestamp_data = SeriesMetricsData(
    'Timestamps', EXAMPLE_TIMESTAMPS, 'seconds', None)
id_data = SeriesMetricsData('Actor IDs', EXAMPLE_IDS).with_unit(
    'UUID').with_index_data(timestamp_data)
labels = SeriesMetricsData('Labels').with_series(
    EXAMPLE_LABELS).with_unit('Category').with_index_data(timestamp_data)
detections = SeriesMetricsData('Detections').with_series(
    EXAMPLE_DETECTIONS).with_unit('Category').with_index_data(timestamp_data)
remapped_detections = detections.map(
    lambda series, index: series[index] if not series[index].endswith(
        '_VEHICLE') else series[index][:-len('_VEHICLE')],
    'Remapped detections',
    detections.unit
)

grouped_labels = labels.group_by(id_data)
grouped_detections = remapped_detections.group_by(id_data)

states_over_time_metric = StatesOverTimeMetric('Labels vs detections')
states_over_time_metric = (states_over_time_metric
                           .with_description('Plot of category labels vs detections, grouped by actor')
                           .with_states_over_time_data([grouped_labels, grouped_detections])
                           .with_legend_series_names(["Labels", "Detections"])
                           .with_blocking(True)
                           .with_should_display(True)
                           .with_status(MetricStatus.Value('NOT_APPLICABLE_METRIC_STATUS'))
                           .with_states_set(EXAMPLE_STATES_SET)
                           .with_importance(MetricImportance.Value("HIGH_IMPORTANCE"))
                           )

label_statuses = grouped_labels.map(
    lambda s, i, c: MetricStatus.Value('NOT_APPLICABLE_METRIC_STATUS'),
    'Label pass/fail'
)

detections_statuses = grouped_detections.map(
    lambda series, index, cat: MetricStatus.Value(
        'PASSED_METRIC_STATUS') if series[index] == grouped_labels.category_to_series[cat][index] else MetricStatus.Value('FAILED_METRIC_STATUS'),
    'Detection pass/fail'
)

states_over_time_metric = (
    states_over_time_metric
    .append_statuses_over_time_data(label_statuses)
    .append_statuses_over_time_data(detections_statuses)
)

print(states_over_time_metric)
