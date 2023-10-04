from __future__ import annotations

from dataclasses import dataclass
import uuid
from typing import Dict, List, Optional

from resim.metrics.proto import metrics_pb2
from resim.metrics.proto.metrics_pb2 import MetricStatus, MetricImportance
import numpy as np

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
    pass

  def add_metrics_data(self, data: MetricsData) -> MetricsData:
    # Only adds if type is valid for job metrics data - i.e. can be packed.
    # Also adds index
    pass

  def add_metric(self, metric: Metric) -> Metric:
    pass

  def pack(self) -> metrics_pb2.JobMetrics:
    pass

# Metrics representation

@dataclass(init=False, repr=True)
class Metric:
  id: uuid.UUID
  name: str
  description: str
  parent_id: uuid.UUID

  status: MetricStatus
  importance: MetricImportance

  should_display: Optional[bool]
  blocking: Optional[bool]


@dataclass(init=False, repr=True)
class ScalarMetric(Metric):
  value: float

  failure_definition: Optional[DoubleFailureDefinition]
  unit: Optional[str]

  def __init__(self,
               name: str,
               description: str,
               parent_id: uuid.UUID,
               status: MetricStatus,
               importance: MetricImportance,
               value: float,
               failure_definition: DoubleFailureDefinition,
               unit: Optional[str] = None,
               should_display: Optional[bool] = None,
               blocking: Optional[bool] = None):
    self.id = uuid.UUID()
    pass

@dataclass(init=False, repr=True)
class DoubleOverTimeMetric(Metric):
  doubles_over_time: MetricsData
  statuses_over_time: MetricsData

  failure_definition: DoubleFailureDefinition

  start_time: Optional[Timestamp]
  end_time: Optional[Timestamp]

  y_axis_name: Optional[str]
  legend_series_names: Optional[List[str]]

  def __init__(self, 
               name: str,
               description: str,
               parent_id: uuid.UUID,
               status: MetricStatus,
               importance: MetricImportance,
               doubles_over_time: MetricsData,
               statues_over_time: MetricsData,
               failure_definition: Optional[DoubleFailureDefinition] = None,
               start_time: Optional[Timestamp] = None,
               end_time: Optional[Timestamp] = None,
               y_axis_name: Optional[str] = None,
               legend_series_name: Optional[List[str]] = None,
               blocking: Optional[bool] = None,
               should_display: Optional[bool] = None):
    self.id = uuid.UUID()
    pass

# Data representation

@dataclass(init=False, repr=True)
class MetricsData:
  id: uuid.UUID
  name: str
  unit: Optional[str]
  index_data: Optional[MetricsData]


@dataclass(init=False, repr=True)
class SeriesMetricsData(MetricsData):
  series: np.ndarray # normally, dtype='object'

  def group_by(self, SeriesMetricsData) -> GroupedMetricsData:
    # also handles appropriate grouping of index.
    pass

  def __init__(self,
               name: str,
               series: np.ndarray,
               unit: Optional[str] = None,
               index_data: Optional[MetricsData] = None):
    pass

@dataclass(init=False, repr=True)
class GroupedMetricsData(MetricsData):
  category_to_series: Dict[str, np.ndarray] # normally, dtype='object'

  def __init__(self,
               name: str,
               category_to_series: Dict[str, np.ndarray],
               unit: Optional[str] = None,
               index_data: Optional[MetricsData] = None):
    pass

print('Reached end of file.')
