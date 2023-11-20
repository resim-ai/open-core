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
            order=None,
            value=24.0,
            failure_definition=None,
            unit="")

        msg = metric.pack()
        self.assertEqual(uuid.UUID(msg.metric_id.id.data), metric.id)
        self.assertEqual(msg.name, metric.name)
         

if __name__ == "__main__":
    unittest.main()
