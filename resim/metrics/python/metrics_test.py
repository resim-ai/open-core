from __future__ import annotations


import uuid
import unittest
import random

import numpy as np

from resim.metrics.python import metrics_writer

from resim.metrics.proto import metrics_pb2
from resim.metrics.proto.metrics_pb2 import MetricStatus, MetricImportance
from resim.utils.proto import uuid_pb2
from google.protobuf import timestamp_pb2
from resim.metrics.python.metrics_utils import Timestamp, DoubleFailureDefinition, HistogramBucket, pack_uuid_to_proto, pack_series_to_proto

from resim.metrics.python.metrics_writer import ResimMetricsWriter, GroupedMetricsData, SeriesMetricsData

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
EXAMPLE_DETECTIONS_SET = ['CAR_VEHICLE', 'TRUCK_VEHICLE',
                          'BIKE_VEHICLE', 'PEDESTRIAN_VEHICLE', 'DEBRIS_VEHICLE', 'UNKNOWN']
EXAMPLE_FAILURE_STATES = ['UNKNOWN']
EXAMPLE_LEGEND_SERIES_NAMES = ['Labels', 'Detections']
EXAMPLE_STATUSES = ['PASSED_METRIC_STATUS', 'FAILED_METRIC_STATUS']

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
    [MetricStatus.Value(rd.choice(EXAMPLE_STATUSES)) for _ in EXAMPLE_IDS])


FIRST_VEHICLE_MASK = (EXAMPLE_IDS == EXAMPLE_ID_SET[0])
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
        METRIC_LEGEND_NAME = "Scalar legend name"
        METRIC_FAILURE_DEFINITION = DoubleFailureDefinition(
            fails_above=3.0, fails_below=8.0)
        METRIC_DESCRIPTION = "Description"
        METRIC_BLOCKING = True
        METRIC_DISPLAY = True
        METRIC_IMPORTANCE = metrics_pb2.MetricImportance.Value(
            'HIGH_IMPORTANCE')
        METRIC_STATUS = metrics_pb2.MetricStatus.Value('PASSED_METRIC_STATUS')
        METRIC_VALUE = 5.0

        scalar_metric = (
            self.writer
            .add_scalar_metric(METRIC_NAME)
            .with_failure_definition(METRIC_FAILURE_DEFINITION)
            .with_unit(METRIC_UNIT)
            .with_description(METRIC_DESCRIPTION)
            .with_blocking(METRIC_BLOCKING)
            .with_should_display(METRIC_DISPLAY)
            .with_importance(METRIC_IMPORTANCE)
            .with_status(METRIC_STATUS)
            .with_value(METRIC_VALUE)
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
        self.assertEqual(metric_base.importance, METRIC_IMPORTANCE)
        self.assertEqual(metric_base.status, METRIC_STATUS)
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

        double_over_time_metric = (
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
            output.metrics_msg.job_level_metrics.metrics[0].metric_values.double_over_time_metric_values)

        self.assertEqual(len(metric_values.doubles_over_time_data_id), 1)
        self.assertIn(
            uuid.UUID(metric_values.doubles_over_time_data_id[0].id.data), output.packed_ids)
        self.assertEqual(len(metric_values.statuses_over_time_data_id), 1)
        self.assertIn(
            uuid.UUID(metric_values.statuses_over_time_data_id[0].id.data), output.packed_ids)

        self.assertEqual(len(metric_values.failure_definition), 1)
        self.assertEqual(
            metric_values.failure_definition[0].fails_above, METRIC_FAILURE_DEFINITION.fails_above)
        self.assertEqual(
            metric_values.failure_definition[0].fails_below, METRIC_FAILURE_DEFINITION.fails_below)

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

    def test_bar_chart_metric(self):
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

        bar_chart_metric = (
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
            uuid.UUID(metric_values.values_data_id[0].id.data), output.packed_ids)
        self.assertEqual(len(metric_values.statuses_data_id), 1)
        self.assertIn(
            uuid.UUID(metric_values.statuses_data_id[0].id.data), output.packed_ids)

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

    def test_histogram_metric(self):
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

        histogram_metric = (
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
            uuid.UUID(metric_values.statuses_data_id.id.data), output.packed_ids)

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

    def test_line_plot_metrics(self):
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

        histogram_metric = (
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

        metric_values = output.metrics_msg.job_level_metrics.metrics[0].metric_values.line_plot_metric_values

        self.assertEqual(len(metric_values.x_doubles_data_id), 2)
        self.assertEqual(len(metric_values.y_doubles_data_id), 2)
        self.assertEqual(len(metric_values.statuses_data_id), 2)
        self.assertEqual(len(metric_values.legend_series_names), 2)

        for i in range(2):
            self.assertIn(uuid.UUID(metric_values.x_doubles_data_id[i].id.data), output.packed_ids)
            self.assertIn(uuid.UUID(metric_values.y_doubles_data_id[i].id.data), output.packed_ids)
            self.assertIn(uuid.UUID(metric_values.statuses_data_id[i].id.data), output.packed_ids)
            self.assertEqual(metric_values.legend_series_names[i], legend_series_names[i])

        self.assertEqual(metric_values.x_axis_name, METRIC_X_AXIS)
        self.assertEqual(metric_values.y_axis_name, METRIC_Y_AXIS)
    
    def test_double_summary_metrics(self) -> None:
        pass

    def tearDown(self) -> None:
        pass


# This is the entry point of the unittest script
if __name__ == '__main__':
    unittest.main()
