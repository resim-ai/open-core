# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""Unit tests for unpack_metrics.py"""

import uuid
import unittest

import resim.metrics.python.unpack_metrics as um
import resim.metrics.proto.generate_test_metrics as gtm
import resim.metrics.python.metrics_writer as mw
import resim.metrics.proto.validate_metrics_proto as vmp


class UnpackMetricsTest(unittest.TestCase):
    """The unit test case"""

    def test_round_trip(self) -> None:
        """Test that pack-unpack-pack leaves the set of metrics and metrics data unchanged"""
        test_metrics = gtm.generate_test_metrics()

        unpacked = um.unpack_metrics(
            metrics=test_metrics.job_level_metrics.metrics,
            metrics_data=test_metrics.metrics_data,
            events=test_metrics.events)

        writer = mw.ResimMetricsWriter(
            job_id=uuid.UUID(
                test_metrics.job_id.id.data))
        for metrics_data in unpacked.metrics_data:
            writer.add_metrics_data(metrics_data)

        for metric in unpacked.metrics:
            writer.add_metric(metric)

        for event in unpacked.events:
            writer.base_add_event(event)

        repacked = writer.write().metrics_msg
        vmp.validate_job_metrics(repacked)

        reunpacked = um.unpack_metrics(
            metrics=repacked.job_level_metrics.metrics,
            metrics_data=repacked.metrics_data,
            events=repacked.events)

        reunpacked_metrics_ids = {metric.id for metric in reunpacked.metrics}
        reunpacked_metrics_data_ids = {
            metric_data.id for metric_data in reunpacked.metrics_data}
        repacked_event_ids = {event.id for event in reunpacked.events}
        unpacked_metrics_ids = {metric.id for metric in unpacked.metrics}
        unpacked_metrics_data_ids = {
            metric_data.id for metric_data in unpacked.metrics_data}
        unpacked_event_ids = {event.id for event in unpacked.events}

        self.assertEqual(unpacked_metrics_ids, reunpacked_metrics_ids)
        self.assertEqual(
            unpacked_metrics_data_ids,
            reunpacked_metrics_data_ids)
        self.assertEqual(unpacked_event_ids,repacked_event_ids)
        
        self.assertEqual(unpacked.metrics, reunpacked.metrics)
        self.assertEqual(unpacked.events, reunpacked.events)


if __name__ == "__main__":
    unittest.main()
