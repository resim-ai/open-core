# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from __future__ import annotations

import typing
import uuid
from dataclasses import dataclass
from typing import Dict, Set

from resim.metrics.proto.metrics_pb2 import MetricStatus as ProtoMetricStatus
from resim.metrics.python.metrics import (
    BarChartMetric,
    BaseMetricsData,
    BaseMetricsDataT,
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
    MetricStatus,
    MetricT,
    PlotlyMetric,
    ScalarMetric,
    SeriesMetricsData,
    StatesOverTimeMetric,
    TextMetric,
)
from resim.metrics.python.metrics_utils import ResimMetricsOutput, pack_uuid_to_proto


@dataclass(init=False, repr=True, kw_only=True)
class ResimMetricsWriter:
    job_id: uuid.UUID
    metrics: Dict[uuid.UUID, Metric]
    metrics_data: Dict[uuid.UUID, BaseMetricsData]
    events: Dict[uuid.UUID, Event]
    metrics_data_names: Set[str]
    event_names: Set[str]

    def __init__(self, job_id: uuid.UUID):
        self.job_id = job_id
        self.metrics = {}
        self.metrics_data = {}
        self.events = {}
        self.metrics_data_names = set()
        self.event_names = set()

    def add_metrics_data(self, data: BaseMetricsDataT) -> BaseMetricsDataT:
        assert data.name not in self.metrics_data_names
        self.metrics_data_names.add(data.name)
        self.metrics_data[data.id] = data
        return data

    def add_metric(self, metric: MetricT) -> MetricT:
        self.metrics[metric.id] = metric
        return metric

    def base_add_event(self, event: Event) -> Event:
        """Given an existent event, add to the writer.

        Args:
            event (Event): the event to add

        Returns:
            Event: the added event, for chaining
        """
        assert event.name not in self.event_names
        self.event_names.add(event.name)
        self.events[event.id] = event
        return event

    def add_event(self, name: str) -> Event:
        """Create an add an event with the given name.

        Args:
            name (str): the name of the event to add

        Returns:
            Event: the added event, for chaining
        """
        event = Event(name=name)
        return self.base_add_event(event)

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

    def add_plotly_metric(self, name: str) -> PlotlyMetric:
        metric = PlotlyMetric(name=name, parent_job_id=self.job_id)
        self.add_metric(metric)
        return metric

    def add_image_metric(self, name: str) -> ImageMetric:
        metric = ImageMetric(name=name, parent_job_id=self.job_id)
        self.add_metric(metric)
        return metric

    def add_image_list_metric(self, name: str) -> ImageListMetric:
        metric = ImageListMetric(name=name, parent_job_id=self.job_id)
        self.add_metric(metric)
        return metric

    def add_text_metric(self, name: str) -> TextMetric:
        metric = TextMetric(name=name, parent_job_id=self.job_id)
        self.add_metric(metric)
        return metric

    def add_batchwise_bar_chart_metric(self, name: str) -> BatchwiseBarChartMetric:
        metric = BatchwiseBarChartMetric(name=name, parent_job_id=self.job_id)
        self.add_metric(metric)
        return metric

    def add_external_file_metrics_data(self, name: str) -> ExternalFileMetricsData:
        metrics_data = ExternalFileMetricsData(name=name)
        self.add_metrics_data(metrics_data)
        return metrics_data

    def write(
        self, metrics_status_override: typing.Optional[MetricStatus] = None
    ) -> ResimMetricsOutput:
        output = ResimMetricsOutput()

        for i, metric in enumerate(self.metrics.values()):
            if metric.order is None:
                metric.order = float(i)
            metric.recursively_pack_into(output)

        for metric_data in self.metrics_data.values():
            metric_data.recursively_pack_into(output)

        for event in self.events.values():
            event.recursively_pack_into(output)

        packed_job_id = pack_uuid_to_proto(self.job_id)
        output.metrics_msg.job_id.id.CopyFrom(packed_job_id)

        if metrics_status_override is not None:
            metrics_status = metrics_status_override.value
        else:
            fail_block = any(
                metric.status == ProtoMetricStatus.Value("FAIL_BLOCK_METRIC_STATUS")
                for metric in output.metrics_msg.job_level_metrics.metrics
            )

            fail_warn = any(
                metric.status == ProtoMetricStatus.Value("FAIL_WARN_METRIC_STATUS")
                for metric in output.metrics_msg.job_level_metrics.metrics
            )

            if fail_block:
                metrics_status = ProtoMetricStatus.Value("FAIL_BLOCK_METRIC_STATUS")
            elif fail_warn:
                metrics_status = ProtoMetricStatus.Value("FAIL_WARN_METRIC_STATUS")
            else:
                metrics_status = ProtoMetricStatus.Value("PASSED_METRIC_STATUS")

        output.metrics_msg.metrics_status = metrics_status
        output.metrics_msg.job_level_metrics.metrics_status = metrics_status
        return output
