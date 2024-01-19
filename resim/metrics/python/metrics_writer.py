# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from __future__ import annotations

from dataclasses import dataclass
import uuid
from typing import Dict, Set

from resim.metrics.proto.metrics_pb2 import MetricStatus
from resim.metrics.python.metrics_utils import ResimMetricsOutput, pack_uuid_to_proto
from resim.metrics.python.metrics import (
    ScalarMetric,
    BarChartMetric,
    LinePlotMetric,
    DoubleSummaryMetric,
    DoubleOverTimeMetric,
    HistogramMetric,
    StatesOverTimeMetric,
    GroupedMetricsData,
    SeriesMetricsData,
    Metric,
    MetricsData,
    MetricsDataT,
    MetricT)


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

        for i, metric in enumerate(self.metrics.values()):
            if metric.order is None:
                metric.order = float(i)
            metric.recursively_pack_into(output)

        for metric_data in self.metrics_data.values():
            metric_data.recursively_pack_into(output)

        packed_job_id = pack_uuid_to_proto(self.job_id)
        output.metrics_msg.job_id.id.CopyFrom(packed_job_id)

        failed = any(metric.status == MetricStatus.Value("FAILED_METRIC_STATUS")
                      for metric in output.metrics_msg.job_level_metrics.metrics)
        metrics_status = (
            MetricStatus.Value("FAILED_METRIC_STATUS")
            if failed
            else MetricStatus.Value("PASSED_METRIC_STATUS"))
        output.metrics_msg.metrics_status = metrics_status
        output.metrics_msg.job_level_metrics.metrics_status = metrics_status
        return output
