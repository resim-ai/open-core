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

    def test_doubles_over_time(self):
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
            .append_doubles_over_time_data(doubles_over_time_metrics_data, "Doubles legend name")
            .append_statuses_over_time_data(statuses_over_time_metrics_data)
        )

        output = self.writer.write()
        self.assertEqual(len(output.packed_ids), 4)
        self.assertEqual(len(output.metrics_msg.job_level_metrics.metrics), 1)
        self.assertEqual(len(output.metrics_msg.metrics_data), 3)

        metric_values = (
            output.metrics_msg.job_level_metrics.metrics[0].metric_values.double_over_time_metric_values)
        print(metric_values)

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

    # Add as many test cases as you need

    def tearDown(self):
        # This method will be called after each test
        # You can clean up resources here, if necessary
        pass


# This is the entry point of the unittest script
if __name__ == '__main__':
    unittest.main()
