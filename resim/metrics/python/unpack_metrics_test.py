# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""Unit tests for unpack_metrics.py"""

import unittest
import resim.metrics.python.unpack_metrics as um

import resim.metrics.proto.generate_test_metrics as gtm

class UnpackMetricsTest(unittest.TestCase):
    """The unit test case"""
    def test_unpack_metrics(self):
        test_metrics = gtm.generate_test_metrics()

        unpacked = um.unpack_metrics(metrics=test_metrics.job_level_metrics.metrics,
                                     metrics_data=test_metrics.metrics_data)


if __name__ == "__main__":
    unittest.main()
