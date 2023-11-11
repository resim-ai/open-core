from __future__ import annotations

from dataclasses import dataclass
import uuid
from typing import Dict, Set

from resim.metrics.python.metrics_utils import ResimMetricsOutput
from resim.metrics.python.metrics import (
  ScalarMetric, BarChartMetric, LinePlotMetric, DoubleSummaryMetric, 
  DoubleOverTimeMetric, HistogramMetric, StatesOverTimeMetric, GroupedMetricsData, 
  SeriesMetricsData, Metric, MetricsData, MetricsDataT, MetricT)

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

    def add_metrics_data(self, data: MetricsDataT) -> MetricsDataT:
        assert data.name not in self.names
        self.names.add(data.name)
        self.metrics_data[data.id] = data
        return data

    def add_metric(self, metric: MetricT) -> MetricT:
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
        metric = StatesOverTimeMetric(name=name, parent_job_id=self.job_id)
        self.add_metric(metric)
        return metric

    def add_double_over_time_metric(self, name: str) -> DoubleOverTimeMetric:
        metric = DoubleOverTimeMetric(name=name, parent_job_id=self.job_id)
        self.add_metric(metric)
        return metric

    def add_bar_chart_metric(self, name: str) -> BarChartMetric:
        metric = BarChartMetric(name=name, parent_job_id=self.job_id)
        self.add_metric(metric)
        return metric

    def add_histogram_metric(self, name: str) -> HistogramMetric:
        metric = HistogramMetric(name=name, parent_job_id=self.job_id)
        self.add_metric(metric)
        return metric

    def add_line_plot_metric(self, name: str) -> LinePlotMetric:
        metric = LinePlotMetric(name=name, parent_job_id=self.job_id)
        self.add_metric(metric)
        return metric

    def add_scalar_metric(self, name: str) -> ScalarMetric:
        metric = ScalarMetric(name=name, parent_job_id=self.job_id)
        self.add_metric(metric)
        return metric

    def add_double_summary_metric(self, name: str) -> DoubleSummaryMetric:
        metric = DoubleSummaryMetric(name=name, parent_job_id=self.job_id)
        self.add_metric(metric)
        return metric

    def write(self) -> ResimMetricsOutput:
        output = ResimMetricsOutput()

        for metric in self.metrics.values():
            metric.recursively_pack_into(output)

        for metric_data in self.metrics_data.values():
            metric_data.recursively_pack_into(output)

        return output
