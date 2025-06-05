# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Unit tests for validate_metrics_proto.
"""

import copy
import unittest
import uuid

import resim.metrics.proto.generate_test_metrics as gtm
import resim.metrics.proto.metrics_pb2 as mp
import resim.metrics.proto.validate_metrics_proto as vmp


def _make_id_bad(job_metrics: mp.JobMetrics) -> mp.JobMetrics:
    """
    Make the job_metrics invalid by setting its job_id data to an invalid hex
    string for a UUID
    """
    result = copy.deepcopy(job_metrics)
    result.job_id.id.data = "foo"
    return result


def _make_data_empty(job_metrics: mp.JobMetrics) -> mp.JobMetrics:
    """
    Make the job_metrics invalid by emptying out the data.
    """
    result = copy.deepcopy(job_metrics)
    del result.metrics_data[:]
    return result


class ValidateMetricsProtoTest(unittest.TestCase):
    """
    Unit test fixture for validate_metrics_proto()
    """

    def setUp(self) -> None:
        """
        Set up our valid test metrics data.
        """
        self._warn_metrics = gtm.generate_test_metrics(False)
        self._block_metrics = gtm.generate_test_metrics(True)

    def test_valid_job_metrics(self) -> None:
        """
        Test that our validator works on our valid test data.
        """
        vmp.validate_job_metrics(self._warn_metrics)
        vmp.validate_job_metrics(self._block_metrics)

    def test_invalid_job_metrics(self) -> None:
        """
        Test that our validator fails on invalidated test data.
        """
        bad_id = _make_id_bad(self._warn_metrics)
        with self.assertRaises(vmp.InvalidMetricsException):
            vmp.validate_job_metrics(bad_id)

        data_empty = _make_data_empty(self._warn_metrics)
        with self.assertRaises(vmp.InvalidMetricsException):
            vmp.validate_job_metrics(data_empty)

    def test_invalid_event_metric(self) -> None:
        """
        Test that the validator fails when a metric used in an event isn't tagged
        or doesn't exist
        """
        for expected_event in [True, False]:
            bad_event_job_proto = gtm.generate_bad_events(expect_event=expected_event)
            with self.assertRaises(vmp.InvalidMetricsException):
                vmp.validate_job_metrics(bad_event_job_proto)

    def test_valid_duplicate_metric_names(self) -> None:
        """
        Test that the validator works when duplicate metrics are split between non-event and event metrics
        """
        metrics = gtm.generate_test_metrics(False)

        # Create two metrics with the same name
        metric1 = metrics.job_level_metrics.metrics[0]
        metric1.event_metric = False

        metric2 = copy.deepcopy(metric1)
        metric2.metric_id.id.data = str(uuid.uuid4())  # Give it a new ID
        metric2.event_metric = True
        metrics.job_level_metrics.metrics.append(metric2)

        metric3 = copy.deepcopy(metric1)
        metric3.metric_id.id.data = str(uuid.uuid4())  # Give it a new ID
        metric3.event_metric = True
        metrics.job_level_metrics.metrics.append(metric3)

        # Add one metric to each event
        metrics.events[0].metrics.append(metric2.metric_id)
        metrics.events[1].metrics.append(metric3.metric_id)

        vmp.validate_job_metrics(metrics)

    def test_duplicate_metric_names_in_event(self) -> None:
        """
        Test that the validator fails when an event contains metrics with duplicate names
        """
        metrics = gtm.generate_test_metrics(False)

        # Create two metrics with the same name
        metric1 = metrics.job_level_metrics.metrics[0]
        metric1.event_metric = True
        metric2 = copy.deepcopy(metric1)
        metric2.metric_id.id.data = str(uuid.uuid4())  # Give it a new ID
        metric2.event_metric = True
        metrics.job_level_metrics.metrics.append(metric2)

        # Add both metrics to the event
        metrics.events[0].metrics.append(metric1.metric_id)
        metrics.events[0].metrics.append(metric2.metric_id)

        with self.assertRaises(vmp.InvalidMetricsException) as context:
            vmp.validate_job_metrics(metrics)
        self.assertIn(
            "Event metric names must be unique within the event", str(context.exception)
        )

    def test_invalid_event_timestamps(self) -> None:
        """
        Test that the validator fails when a set of events has:
        - an un-set timestamp_type
        - conflicting timestamp types
        """
        basic_job_proto = gtm.generate_test_metrics(False)
        vmp.validate_job_metrics(basic_job_proto)

        # test that setting the timestamp type to none fails:
        basic_job_proto.events[0].timestamp_type = mp.NO_TYPE
        with self.assertRaises(vmp.InvalidMetricsException):
            vmp.validate_job_metrics(basic_job_proto)

        # validate that when the types are different we get a failure:
        basic_job_proto.events[0].timestamp_type = mp.ABSOLUTE_TIMESTAMP
        with self.assertRaises(vmp.InvalidMetricsException):
            vmp.validate_job_metrics(basic_job_proto)

    def test_invalid_key_value_tags(self) -> None:
        """
        Test empty key/value tags are rejected
        """
        metrics = gtm.generate_test_metrics(True)

        # key empty
        metrics.job_level_metrics.metrics[0].tags.add(key="", value="value")

        with self.assertRaises(vmp.InvalidMetricsException):
            vmp.validate_job_metrics(metrics)

        # value empty
        metrics.job_level_metrics.metrics[0].tags[0].key = "key"
        metrics.job_level_metrics.metrics[0].tags[0].value = ""

        with self.assertRaises(vmp.InvalidMetricsException):
            vmp.validate_job_metrics(metrics)

        # both empty
        metrics.job_level_metrics.metrics[0].tags[0].key = ""
        metrics.job_level_metrics.metrics[0].tags[0].value = ""

        with self.assertRaises(vmp.InvalidMetricsException):
            vmp.validate_job_metrics(metrics)

    def test_nan_validation(self) -> None:
        """
        Test that NaN values in metrics data are rejected
        """
        metrics = gtm.generate_test_metrics(True)

        # Test NaN in regular series
        scalar_metric = next(
            metric
            for metric in metrics.job_level_metrics.metrics
            if metric.name == "Ambient Temperature"
        )
        scalar_metric.metric_values.scalar_metric_values.value = float("nan")

        with self.assertRaises(vmp.InvalidMetricsException):
            vmp.validate_job_metrics(metrics)


if __name__ == "__main__":
    unittest.main()
