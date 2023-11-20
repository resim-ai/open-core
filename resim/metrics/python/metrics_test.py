# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Unit tests for metrics.py.
"""

import copy
import uuid
import unittest
import resim.metrics.python.metrics as metrics
from resim.metrics.python.metrics_utils import MetricStatus, MetricImportance
import resim.metrics.proto.metrics_pb2 as mp


class MetricsTest(unittest.TestCase):
    def test_metric_eq(self) -> None:
        # SETUP
        job_id = uuid.uuid4()
        metric = metrics.ScalarMetric(
            "test_metric",
            "A metric for testing",
            MetricStatus.PASSED_METRIC_STATUS,
            MetricImportance.ZERO_IMPORTANCE,
            should_display=True,
            blocking=False,
            parent_job_id=job_id,
            order=None,
            value=24.0,
            failure_definition=None,
            unit="")

        # Test equality
        self.assertEqual(metric, metric)

        # Different type
        self.assertNotEqual(metric, 3)

        # Non-existent id
        metric_with_none_id = copy.copy(metric)
        metric_with_none_id.id = None
        with self.assertRaises(AssertionError):
            metric_with_none_id == metric
        with self.assertRaises(AssertionError):
            metric == metric_with_none_id

        # Different id
        metric_with_diff_id = copy.copy(metric)
        metric_with_diff_id.id = uuid.uuid4()
        self.assertNotEqual(metric_with_diff_id, metric)
        self.assertNotEqual(metric, metric_with_diff_id)

    def test_metric_pack(self):
        # SETUP
        job_id = uuid.uuid4()
        metric = metrics.ScalarMetric(
            "test_metric",
            "A metric for testing",
            MetricStatus.PASSED_METRIC_STATUS,
            MetricImportance.ZERO_IMPORTANCE,
            should_display=True,
            blocking=False,
            parent_job_id=job_id,
            order=1.5,
            value=24.0,
            failure_definition=None,
            unit="")

        msg = metric.pack()
        self.assertEqual(uuid.UUID(msg.metric_id.id.data), metric.id)
        self.assertEqual(msg.name, metric.name)
        self.assertEqual(msg.description, metric.description)
        self.assertEqual(msg.status, metric.status.value)
        self.assertEqual(msg.importance, metric.importance.value)
        self.assertEqual(msg.should_display, metric.should_display)
        self.assertEqual(msg.blocking, metric.blocking)
        self.assertEqual(uuid.UUID(msg.job_id.id.data), metric.parent_job_id)
        self.assertEqual(msg.order, metric.order)

        optional_attr_list = [
            ("description", "description"),
            ("status", "status"),
            ("importance", "importance"),
            ("should_display", "should_display"),
            ("blocking", "blocking"),
            ("parent_job_id", "job_id"),
            ("order", "order")]

        default_values = mp.Metric()

        for unpacked_attr, packed_attr in optional_attr_list:
            modified_metric = copy.copy(metric)
            setattr(modified_metric, unpacked_attr, None)
            msg = modified_metric.pack()
            self.assertEqual(
                getattr(
                    msg, packed_attr), getattr(
                    default_values, packed_attr))


if __name__ == "__main__":
    unittest.main()
