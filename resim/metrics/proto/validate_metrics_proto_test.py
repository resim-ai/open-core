# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import copy
import unittest

import resim.metrics.proto.validate_metrics_proto as vmp
import resim.metrics.proto.metrics_pb2 as mp
import resim.metrics.proto.generate_test_metrics as gtm

import google.protobuf.json_format as jf


def _make_all_blocking(job_metrics: mp.JobMetrics):
    result = copy.deepcopy(job_metrics)
    for metric in result.job_level_metrics.metrics:
        metric.blocking = True
    return result


def _make_id_bad(job_metrics: mp.JobMetrics):
    result = copy.deepcopy(job_metrics)
    result.job_id.id.data = "foo"
    return result


class ValidateMetricsProtoTest(unittest.TestCase):
    def setUp(self):
        self._valid_metrics = gtm.generate_test_metrics()

    def test_valid_job_metrics(self):
        vmp.validate_job_metrics(self._valid_metrics)

        all_blocking = _make_all_blocking(self._valid_metrics)
        vmp.validate_job_metrics(all_blocking)

        bad_id = _make_id_bad(self._valid_metrics)
        with self.assertRaises(vmp.InvalidMetricsException):
            vmp.validate_job_metrics(bad_id)


if __name__ == '__main__':
    unittest.main()
