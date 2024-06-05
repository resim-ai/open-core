from __future__ import annotations

import uuid
import unittest
import random
from typing import List, Optional

import numpy as np

from google.protobuf.struct_pb2 import Struct
from resim.metrics.python.metrics_utils import (
    Timestamp,
    DoubleFailureDefinition,
    HistogramBucket,
    MetricStatus,
    MetricImportance
)
from resim.metrics.python.metrics import SeriesMetricsData, GroupedMetricsData
from resim.metrics.python.metrics_writer import ResimMetricsWriter

rd = random.Random()
rd.seed(194842)


def consistent_uuid() -> uuid.UUID:
    return uuid.UUID(int=rd.getrandbits(128), version=4)


# Fake data, assume this is read in
NUM_ACTORS = 10
EXAMPLE_ID_SET = [consistent_uuid() for i in range(NUM_ACTORS)]
EXAMPLE_COUNTS = [10 * (i + 1) for i in range(NUM_ACTORS)]
EXAMPLE_STATES_SET = ['CAR', 'TRUCK', 'BIKE',
                      'PEDESTRIAN', 'DEBRIS', 'UNKNOWN']
EXAMPLE_DETECTIONS_SET = [
    'CAR_VEHICLE',
    'TRUCK_VEHICLE',
    'BIKE_VEHICLE',
    'PEDESTRIAN_VEHICLE',
    'DEBRIS_VEHICLE',
    'UNKNOWN']
EXAMPLE_FAILURE_STATES = ['UNKNOWN']
EXAMPLE_LEGEND_SERIES_NAMES = ['Labels', 'Detections']
EXAMPLE_STATUSES = [
    MetricStatus.PASSED_METRIC_STATUS,
    MetricStatus.FAIL_WARN_METRIC_STATUS]

EXAMPLE_IDS = np.array([EXAMPLE_ID_SET[i] for i in range(
    NUM_ACTORS) for _ in range(EXAMPLE_COUNTS[i])])
EXAMPLE_TIMESTAMPS = np.array(
    [Timestamp(secs=i + 1, nanos=i * 10000) for i, _ in enumerate(EXAMPLE_IDS)])
EXAMPLE_LABELS = np.array([rd.choice(EXAMPLE_STATES_SET)
                          for _ in EXAMPLE_IDS])
EXAMPLE_DETECTIONS = np.array(
    [rd.choice(EXAMPLE_DETECTIONS_SET) for _ in EXAMPLE_IDS])
EXAMPLE_FLOATS = np.array([rd.random() for _ in EXAMPLE_IDS])
EXAMPLE_UUIDS = np.array([consistent_uuid() for _ in EXAMPLE_IDS])
EXAMPLE_STATUSES = np.array(
    [MetricStatus(rd.choice(EXAMPLE_STATUSES)) for _ in EXAMPLE_IDS])


FIRST_VEHICLE_MASK = EXAMPLE_IDS == EXAMPLE_ID_SET[0]
FIRST_VEHICLE_IDS = EXAMPLE_IDS[FIRST_VEHICLE_MASK]
FIRST_VEHICLE_TIMESTAMPS = EXAMPLE_TIMESTAMPS[FIRST_VEHICLE_MASK]
FIRST_VEHICLE_LABELS = EXAMPLE_LABELS[FIRST_VEHICLE_MASK]
FIRST_VEHICLE_DETECTIONS = EXAMPLE_DETECTIONS[FIRST_VEHICLE_MASK]


class TestMetricsWriter(unittest.TestCase):
    def setUp(self) -> None:
        self.job_id = consistent_uuid()
        self.writer = ResimMetricsWriter(self.job_id)

    def test_scalar_metric(self) -> None:
        METRIC_NAME = "Scalar metric"
        METRIC_UNIT = "Scalar unit"
        METRIC_FAILURE_DEFINITION = DoubleFailureDefinition(
            fails_above=3.0, fails_below=8.0)
        METRIC_DESCRIPTION = "Description"
        METRIC_BLOCKING = True
        METRIC_DISPLAY = True
        METRIC_IMPORTANCE = MetricImportance.HIGH_IMPORTANCE
        METRIC_STATUS = MetricStatus.PASSED_METRIC_STATUS
        METRIC_VALUE = 5.0

        (
            self.writer
            .add_scalar_metric(METRIC_NAME)
            .with_failure_definition(METRIC_FAILURE_DEFINITION)
            .with_unit(METRIC_UNIT)
            .with_value(METRIC_VALUE)
            .with_description(METRIC_DESCRIPTION)
            .with_blocking(METRIC_BLOCKING)
            .with_should_display(METRIC_DISPLAY)
            .with_importance(METRIC_IMPORTANCE)
            .with_status(METRIC_STATUS)
        )

        output = self.writer.write()
        self.assertEqual(len(output.packed_ids), 1)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 0)

        metric_base = output.metrics_msg.job_level_metrics.metrics[0]
        metric_values = metric_base.metric_values.scalar_metric_values

        self.assertEqual(metric_base.description, METRIC_DESCRIPTION)
        self.assertEqual(metric_base.blocking, METRIC_BLOCKING)
        self.assertEqual(metric_base.should_display, METRIC_DISPLAY)
        self.assertEqual(metric_base.importance, METRIC_IMPORTANCE.value)
        self.assertEqual(metric_base.status, METRIC_STATUS.value)
        self.assertEqual(metric_base.name, METRIC_NAME)
        self.assertEqual(metric_values.value, METRIC_VALUE)
        self.assertEqual(metric_values.unit, METRIC_UNIT)
        self.assertEqual(metric_values.failure_definition.fails_above,
                         METRIC_FAILURE_DEFINITION.fails_above)

    def test_doubles_over_time(self) -> None:
        METRIC_NAME = "Doubles metric"
        METRIC_UNIT = "Doubles axis"
        METRIC_LEGEND_NAME = "Doubles legend name"
        METRIC_START_TIME = Timestamp(secs=0, nanos=30)
        METRIC_END_TIME = Timestamp(secs=10000, nanos=3000)
        METRIC_FAILURE_DEFINITION = DoubleFailureDefinition(
            fails_above=1.0, fails_below=2.0)

        time_metrics_data = SeriesMetricsData(
            "Time", EXAMPLE_TIMESTAMPS, "Time unit", None
        )
        doubles_over_time_metrics_data = SeriesMetricsData(
            "Doubles", EXAMPLE_FLOATS, "Double unit", time_metrics_data)

        statuses_over_time_metrics_data = SeriesMetricsData(
            "Statuses", EXAMPLE_STATUSES, "Double unit", time_metrics_data
        )

        (
            self.writer
            .add_double_over_time_metric(METRIC_NAME)
            .with_failure_definitions([METRIC_FAILURE_DEFINITION])
            .with_y_axis_name(METRIC_UNIT)
            .with_start_time(METRIC_START_TIME)
            .with_end_time(METRIC_END_TIME)
            .append_doubles_over_time_data(doubles_over_time_metrics_data, METRIC_LEGEND_NAME)
            .append_statuses_over_time_data(statuses_over_time_metrics_data)
        )

        output = self.writer.write()
        self.assertEqual(len(output.packed_ids), 4)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 3)

        metric_values = (
            output
            .metrics_msg
            .job_level_metrics
            .metrics[0]
            .metric_values
            .double_over_time_metric_values)

        self.assertEqual(len(metric_values.doubles_over_time_data_id), 1)
        self.assertIn(
            uuid.UUID(
                metric_values.doubles_over_time_data_id[0].id.data),
            output.packed_ids)
        self.assertEqual(len(metric_values.statuses_over_time_data_id), 1)
        self.assertIn(
            uuid.UUID(
                metric_values.statuses_over_time_data_id[0].id.data),
            output.packed_ids)

        self.assertEqual(len(metric_values.failure_definition), 1)
        self.assertEqual(
            metric_values.failure_definition[0].fails_above,
            METRIC_FAILURE_DEFINITION.fails_above)
        self.assertEqual(
            metric_values.failure_definition[0].fails_below,
            METRIC_FAILURE_DEFINITION.fails_below)

        self.assertEqual(
            metric_values.start_time.nanos, METRIC_START_TIME.nanos
        )
        self.assertEqual(
            metric_values.start_time.seconds, METRIC_START_TIME.secs
        )

        self.assertEqual(
            metric_values.end_time.nanos, METRIC_END_TIME.nanos
        )
        self.assertEqual(
            metric_values.end_time.seconds, METRIC_END_TIME.secs
        )

        self.assertEqual(
            metric_values.y_axis_name, METRIC_UNIT
        )

        self.assertEqual(
            len(metric_values.legend_series_names), 1
        )
        self.assertEqual(
            metric_values.legend_series_names[0], METRIC_LEGEND_NAME
        )

    def test_bar_chart_metric(self) -> None:
        METRIC_NAME = "Bar chart metric metric"
        METRIC_Y_AXIS = "Bar chart y axis"
        METRIC_X_AXIS = "Bar chart x axis"
        METRIC_LEGEND_NAME = "Bar chart legend name"
        METRIC_STACK_BARS = True

        strings_metrics_data = SeriesMetricsData(
            "Actors", EXAMPLE_DETECTIONS, "UUID unit", None
        )
        doubles_over_strings_data = SeriesMetricsData(
            "Doubles", EXAMPLE_FLOATS, "Double unit", strings_metrics_data)

        statuses_over_strings_data = SeriesMetricsData(
            "Statuses", EXAMPLE_STATUSES, "Double unit", strings_metrics_data
        )

        (
            self.writer
            .add_bar_chart_metric(METRIC_NAME)
            .with_y_axis_name(METRIC_Y_AXIS)
            .with_x_axis_name(METRIC_X_AXIS)
            .append_values_data(doubles_over_strings_data, METRIC_LEGEND_NAME)
            .append_statuses_data(statuses_over_strings_data)
            .with_stack_bars(METRIC_STACK_BARS)
        )

        output = self.writer.write()
        self.assertEqual(len(output.packed_ids), 4)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 3)

        metric_values = (
            output.metrics_msg.job_level_metrics.metrics[0].metric_values.bar_chart_metric_values)

        self.assertEqual(len(metric_values.values_data_id), 1)
        self.assertIn(
            uuid.UUID(
                metric_values.values_data_id[0].id.data),
            output.packed_ids)
        self.assertEqual(len(metric_values.statuses_data_id), 1)
        self.assertIn(
            uuid.UUID(
                metric_values.statuses_data_id[0].id.data),
            output.packed_ids)

        self.assertEqual(
            metric_values.y_axis_name, METRIC_Y_AXIS
        )
        self.assertEqual(
            metric_values.x_axis_name, METRIC_X_AXIS
        )
        self.assertEqual(
            metric_values.stack_bars, METRIC_STACK_BARS
        )

        self.assertEqual(
            len(metric_values.legend_series_names), 1
        )
        self.assertEqual(
            metric_values.legend_series_names[0], METRIC_LEGEND_NAME
        )

    def test_histogram_metric(self) -> None:
        METRIC_NAME = "Histogram metric"
        METRIC_X_AXIS = "Histogram x axis"
        METRIC_LOWER_BOUND = 0.0
        METRIC_UPPER_BOUND = 10.0
        METRIC_BUCKETS = [HistogramBucket(
            float(i), float(i + 1)) for i in range(10)]

        doubles_data = SeriesMetricsData(
            "Doubles", EXAMPLE_FLOATS, "Double unit", None
        )

        statuses_data = SeriesMetricsData(
            "Statuses", EXAMPLE_STATUSES, "Double unit", None
        )

        (
            self.writer
            .add_histogram_metric(METRIC_NAME)
            .with_x_axis_name(METRIC_X_AXIS)
            .with_values_data(doubles_data)
            .with_statuses_data(statuses_data)
            .with_lower_bound(METRIC_LOWER_BOUND)
            .with_upper_bound(METRIC_UPPER_BOUND)
            .with_buckets(METRIC_BUCKETS)
        )

        output = self.writer.write()
        self.assertEqual(len(output.packed_ids), 3)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 2)

        metric_values = (
            output.metrics_msg.job_level_metrics.metrics[0].metric_values.histogram_metric_values)
        self.assertIn(
            uuid.UUID(metric_values.values_data_id.id.data), output.packed_ids)
        self.assertIn(
            uuid.UUID(
                metric_values.statuses_data_id.id.data),
            output.packed_ids)

        self.assertEqual(
            metric_values.x_axis_name, METRIC_X_AXIS
        )
        self.assertEqual(
            metric_values.lower_bound, METRIC_LOWER_BOUND
        )
        self.assertEqual(
            metric_values.upper_bound, METRIC_UPPER_BOUND
        )
        for i, bucket in enumerate(METRIC_BUCKETS):
            self.assertEqual(bucket.lower, metric_values.buckets[i].lower)
            self.assertEqual(bucket.upper, metric_values.buckets[i].upper)

    def test_line_plot_metrics(self) -> None:
        METRIC_NAME = "Line plot metric"
        METRIC_X_AXIS = "Line plot x axis"
        METRIC_Y_AXIS = "Line plot y axis"

        doubles_data_one = SeriesMetricsData(
            "Doubles one", EXAMPLE_FLOATS, "Double unit", None
        )
        doubles_data_two = SeriesMetricsData(
            "Doubles two", EXAMPLE_FLOATS[::-1], "Double unit 2", None
        )

        statuses_data_one = SeriesMetricsData(
            "Statuses one", EXAMPLE_STATUSES, "Status unit", None
        )
        statuses_data_two = SeriesMetricsData(
            "Statuses two", EXAMPLE_STATUSES[::-1], "Status unit", None
        )

        legend_series_names = ["Double legend one", "Double legend two"]

        (
            self.writer
            .add_line_plot_metric(METRIC_NAME)
            .with_x_axis_name(METRIC_X_AXIS)
            .with_y_axis_name(METRIC_Y_AXIS)
            .append_series_data(doubles_data_one, doubles_data_two, legend_series_names[0])
            .append_statuses_data(statuses_data_one)
            .append_series_data(doubles_data_two, doubles_data_one, legend_series_names[1])
            .append_statuses_data(statuses_data_two)
        )

        output = self.writer.write()
        self.assertEqual(len(output.packed_ids), 5)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 4)

        metric_values = output.metrics_msg.job_level_metrics.metrics[
            0].metric_values.line_plot_metric_values

        self.assertEqual(len(metric_values.x_doubles_data_id), 2)
        self.assertEqual(len(metric_values.y_doubles_data_id), 2)
        self.assertEqual(len(metric_values.statuses_data_id), 2)
        self.assertEqual(len(metric_values.legend_series_names), 2)

        for i in range(2):
            self.assertIn(
                uuid.UUID(
                    metric_values.x_doubles_data_id[i].id.data),
                output.packed_ids)
            self.assertIn(
                uuid.UUID(
                    metric_values.y_doubles_data_id[i].id.data),
                output.packed_ids)
            self.assertIn(
                uuid.UUID(
                    metric_values.statuses_data_id[i].id.data),
                output.packed_ids)
            self.assertEqual(
                metric_values.legend_series_names[i], legend_series_names[i])

        self.assertEqual(metric_values.x_axis_name, METRIC_X_AXIS)
        self.assertEqual(metric_values.y_axis_name, METRIC_Y_AXIS)

    def test_double_summary_metrics(self) -> None:
        METRIC_NAME = "Double summary metric"
        METRIC_INDEX = 23
        doubles_data = SeriesMetricsData(
            "Doubles", EXAMPLE_FLOATS, "Double unit", None
        )

        (
            self.writer
            .add_double_summary_metric(METRIC_NAME)
            .with_value_data(doubles_data)
            .with_index(METRIC_INDEX)
        )

        output = self.writer.write()

        self.assertEqual(len(output.packed_ids), 2)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 1)

        metric_values = output.metrics_msg.job_level_metrics.metrics[
            0].metric_values.double_metric_values

        self.assertEqual(metric_values.series_index, METRIC_INDEX)

        self.assertFalse(
            metric_values.failure_definition.HasField('fails_above'))
        self.assertFalse(
            metric_values.failure_definition.HasField('fails_below'))

        self.assertEqual(len(
            output.metrics_msg.metrics_data[0].series.doubles.series), len(EXAMPLE_FLOATS))

        for i, x in enumerate(EXAMPLE_FLOATS):
            self.assertEqual(
                output.metrics_msg.metrics_data[0].series.doubles.series[i], x)

    def test_states_over_time_metric(self) -> None:
        METRIC_NAME = 'States over time metric'
        METRIC_DESCRIPTION = 'Plot of labels vs detections, grouped by UUID'
        METRIC_LEGEND_SERIES_NAMES: List[Optional[str]] = [
            'Labels', 'Detections']
        METRIC_BLOCKING = True
        METRIC_SHOULD_DISPLAY = True
        METRIC_STATUS = MetricStatus.FAIL_BLOCK_METRIC_STATUS

        timestamp_data = (SeriesMetricsData('Timestamps')
                          .with_series(EXAMPLE_TIMESTAMPS)
                          .with_unit('seconds'))
        id_data = (SeriesMetricsData('Actor IDs')
                   .with_series([str(i) for i in EXAMPLE_IDS])
                   .with_unit('UUID')
                   .with_index_data(timestamp_data))
        labels = (self.writer.
                  add_series_metrics_data('Labels')
                  .with_series(EXAMPLE_LABELS)
                  .with_unit('Category')
                  .with_index_data(timestamp_data))
        detections = (self.writer
                      .add_series_metrics_data('Detections')
                      .with_series(EXAMPLE_DETECTIONS)
                      .with_unit('Category')
                      .with_index_data(timestamp_data))
        remapped_detections = self.writer.add_metrics_data(detections.map(
            lambda series, index: series[index] if not series[index].endswith(
                '_VEHICLE') else series[index][:-len('_VEHICLE')],
            'Remapped detections',
            detections.unit
        ))

        # Example for a grouped metric, handling data fairly manually
        grouped_labels = self.writer.add_metrics_data(labels.group_by(id_data))
        grouped_detections = self.writer.add_metrics_data(
            remapped_detections.group_by(id_data))

        (
            self.writer
                .add_states_over_time_metric(METRIC_NAME)
                .with_states_over_time_data([grouped_labels, grouped_detections])
                .with_legend_series_names(METRIC_LEGEND_SERIES_NAMES)
                .with_states_set(set(EXAMPLE_STATES_SET))
                .with_failure_states(set())
                .with_description(METRIC_DESCRIPTION)
                .with_blocking(METRIC_BLOCKING)
                .with_should_display(METRIC_SHOULD_DISPLAY)
                .with_status(METRIC_STATUS)
        )

        output = self.writer.write()

        self.assertEqual(len(output.packed_ids), 9)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 8)

        base_metric = output.metrics_msg.job_level_metrics.metrics[0]
        metric_values = base_metric.metric_values.states_over_time_metric_values

        self.assertEqual(base_metric.name, METRIC_NAME)
        self.assertEqual(base_metric.description, METRIC_DESCRIPTION)
        self.assertEqual(base_metric.blocking, METRIC_BLOCKING)
        self.assertEqual(base_metric.should_display, METRIC_SHOULD_DISPLAY)
        self.assertEqual(base_metric.status, METRIC_STATUS.value)
        self.assertEqual(base_metric.order, 0.0)

        self.assertEqual(set(metric_values.states_set),
                         set(EXAMPLE_STATES_SET))
        self.assertEqual(len(metric_values.failure_states), 0)

        self.assertIn(grouped_labels.id, output.packed_ids)
        for data in output.metrics_msg.metrics_data:
            if uuid.UUID(data.metrics_data_id.id.data) == grouped_labels.id:
                self.assertTrue(data.is_per_category)
                self.assertEqual(
                    set(data.series_per_category.category_to_series.keys()),
                    set(grouped_labels.category_to_series.keys()))

    def test_two_metrics(self) -> None:
        METRIC_NAMES = [
            "Double summary metric one",
            "Double summary metric two"]
        METRIC_INDICES = [23, 25]
        doubles_data = SeriesMetricsData(
            "Doubles", EXAMPLE_FLOATS, "Double unit", None
        )
        ORDERS = [1.0, 10.0]

        for name, index, order in zip(METRIC_NAMES, METRIC_INDICES, ORDERS):
            metric = (
                self.writer
                .add_double_summary_metric(name)
                .with_value_data(doubles_data)
                .with_index(index)
            )
            metric.order = order

        output = self.writer.write()

        self.assertEqual(len(output.packed_ids), len(METRIC_NAMES) + 1)
        self.assertEqual(
            len(output.metrics_msg.job_level_metrics.metrics), len(METRIC_NAMES))
        self.assertEqual(len(output.metrics_msg.metrics_data), 1)

        metrics_data_id = output.metrics_msg.metrics_data[0].metrics_data_id.id.data

        for i in range(len(METRIC_NAMES)):
            metric_values = output.metrics_msg.job_level_metrics.metrics[
                i].metric_values.double_metric_values

            self.assertEqual(metric_values.series_index, METRIC_INDICES[i])

            self.assertFalse(
                metric_values.failure_definition.HasField('fails_above'))
            self.assertFalse(
                metric_values.failure_definition.HasField('fails_below'))

            self.assertEqual(
                metric_values.value_data_id.id.data,
                metrics_data_id)

            self.assertEqual(
                output.metrics_msg.job_level_metrics.metrics[i].order,
                ORDERS[i])

        self.assertEqual(len(
            output.metrics_msg.metrics_data[0].series.doubles.series), len(EXAMPLE_FLOATS))

    def test_grouped_metrics_data(self) -> None:
        """Test that we can add a grouped metrics data."""
        NAME = "test_metrics_data"
        metrics_data = self.writer.add_grouped_metrics_data(name=NAME)
        self.assertEqual(type(metrics_data), GroupedMetricsData)
        self.assertEqual(metrics_data.name, NAME)
        self.assertIn(NAME, self.writer.names)
        self.assertIn(metrics_data.id, self.writer.metrics_data)
        self.assertEqual(
            self.writer.metrics_data[metrics_data.id], metrics_data)

    def test_plotly_metric(self) -> None:
        METRIC_NAME = "Plotly metric"
        METRIC_DESCRIPTION = "Description"
        METRIC_BLOCKING = True
        METRIC_DISPLAY = True
        METRIC_IMPORTANCE = MetricImportance.HIGH_IMPORTANCE
        METRIC_STATUS = MetricStatus.PASSED_METRIC_STATUS
        METRIC_DATA = Struct()

        (
            self.writer
            .add_plotly_metric(METRIC_NAME)
            .with_plotly_data(METRIC_DATA)
            .with_description(METRIC_DESCRIPTION)
            .with_blocking(METRIC_BLOCKING)
            .with_should_display(METRIC_DISPLAY)
            .with_importance(METRIC_IMPORTANCE)
            .with_status(METRIC_STATUS)
        )

        output = self.writer.write()
        self.assertEqual(len(output.packed_ids), 1)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 0)

        metric_base = output.metrics_msg.job_level_metrics.metrics[0]
        metric_values = metric_base.metric_values.plotly_metric_values

        self.assertEqual(metric_base.description, METRIC_DESCRIPTION)
        self.assertEqual(metric_base.blocking, METRIC_BLOCKING)
        self.assertEqual(metric_base.should_display, METRIC_DISPLAY)
        self.assertEqual(metric_base.importance, METRIC_IMPORTANCE.value)
        self.assertEqual(metric_base.status, METRIC_STATUS.value)
        self.assertEqual(metric_base.name, METRIC_NAME)
        self.assertEqual(metric_values.json, METRIC_DATA)

    def tearDown(self) -> None:
        pass

# This is the entry point of the unittest script
if __name__ == '__main__':
    unittest.main()
