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

import resim.metrics.proto.validate_metrics_proto as vmp
import resim.metrics.proto.metrics_pb2 as mp
import resim.metrics.proto.generate_test_metrics as gtm

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

    def test_invalid_event_timestamps(self) -> None:
        """
        Test that the validator fails when an event has:
        - both absolute and relative timestamps set
        - has relative set but marked as absolute
        - vice versa
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


if __name__ == '__main__':
    unittest.main()
