# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Unit test for fetch_job_metrics().
"""

import unittest
import uuid
import typing
from dataclasses import dataclass
from unittest.mock import patch

import requests
from resim_python_client.client import AuthenticatedClient

import resim.metrics.fetch_job_metrics as fjm

_TOKEN = "wowwhatacoincidentaltoken"


@dataclass(frozen=True)
class MockMetric:
    """A mock for a resim.metric.proto.Metric protobuf message."""
    name: str
    data: str

@dataclass(frozen=True)
class MockMetricsData:
    """A mock for a resim.metric.proto.MetricsData protobuf message."""
    name: str
    data: str

@dataclass(frozen=True)
class MockJob:
    job_id: str

@dataclass(frozen=True)
class MockListJobsResponse200:
    jobs: list[MockJob]

@dataclass
class MockUnpackedMetrics:
    metrics: list[MockMetric]
    metrics_data: list[MockMetricsData]
    names: set[str]

_JOB_ID_METRICS_URL_MAP = {
    uuid.UUID("1f2441f6-cee2-4078-9729-048810c1da96"):
        [f"https://example.com/metrics_{i}.binproto" for i in range(3)],
    uuid.UUID("f1b4fa78-7ede-46c8-bd8e-deb5681e8010"):
        [f"https://example.com/metrics_{i}.binproto" for i in range(3, 6)],
    uuid.UUID("b0425a97-e773-49b0-b331-6378d0f101a5"):
        [f"https://example.com/metrics_{i}.binproto" for i in range(6, 10)],
}

_JOB_ID_METRICS_DATA_URL_MAP = {
    uuid.UUID("1f2441f6-cee2-4078-9729-048810c1da96"):
        [f"https://example.com/metrics_data_{i}.binproto" for i in range(3)],
    uuid.UUID("f1b4fa78-7ede-46c8-bd8e-deb5681e8010"):
        [f"https://example.com/metrics_data_{i}.binproto" for i in range(3, 6)],
    uuid.UUID("b0425a97-e773-49b0-b331-6378d0f101a5"):
        [f"https://example.com/metrics_data_{i}.binproto" for i in range(6, 10)],
}

_BATCH_IDS = {
    uuid.UUID("b6ff9ce1-a481-4793-a9ea-d607f6efe628"),
    uuid.UUID("0a74e080-7782-4a6c-ac26-248caa575405"),
}

_BATCH_IDS_TO_JOB_IDS_MAP = {
    uuid.UUID("b6ff9ce1-a481-4793-a9ea-d607f6efe628"):
        [uuid.UUID("1f2441f6-cee2-4078-9729-048810c1da96"),
         uuid.UUID("f1b4fa78-7ede-46c8-bd8e-deb5681e8010")],
    uuid.UUID("0a74e080-7782-4a6c-ac26-248caa575405"):
        [uuid.UUID("b0425a97-e773-49b0-b331-6378d0f101a5")],
}

_METRICS_URL_TO_MESSAGE_MAP = {
    f"https://example.com/metrics_{i}.binproto": 
    MockMetric(name = f"metric_{i} name", data=f"metric_{i}")
    for i in range(10)
}

_METRICS_DATA_URL_TO_MESSAGE_MAP = {
    f"https://example.com/metrics_data_{i}.binproto": 
    MockMetricsData(name = f"data_{i} name", data=f"data_{i}")
    for i in range(10)
}

def _mock_fetch_metrics_urls(*,
                             batch_id: uuid.UUID,
                             job_id: uuid.UUID,
                             client: AuthenticatedClient) -> list[str]:
    assert batch_id in _BATCH_IDS
    assert client.token == _TOKEN
    return list(_JOB_ID_METRICS_URL_MAP[job_id])


def _mock_fetch_metrics_data_urls(*,
                                  batch_id: uuid.UUID,
                                  job_id: uuid.UUID,
                                  client: AuthenticatedClient) -> list[str]:
    assert batch_id in _BATCH_IDS
    assert client.token == _TOKEN
    return list(_JOB_ID_METRICS_DATA_URL_MAP[job_id])

def _mock_list_job_ids_by_batch(batch_id: str,
                                *,
                                client: AuthenticatedClient) -> MockListJobsResponse200:
    batch_id_uuid = uuid.UUID(batch_id)

    assert batch_id_uuid in _BATCH_IDS
    assert client.token == _TOKEN
    return MockListJobsResponse200(
        jobs= [MockJob(job_id=str(job_id)) for job_id in _BATCH_IDS_TO_JOB_IDS_MAP[batch_id_uuid]]
    )

# TODO(tknowles): In an ideal world, this would not be mocked out, but this will require us
#                 to use valid messages within our Mock protobuf messages.
def _mock_unpack_metrics(
    *,
    metrics: list[MockMetric],
    metrics_data: list[MockMetricsData]) -> MockUnpackedMetrics:
    return MockUnpackedMetrics(
        metrics=metrics[:],
        metrics_data=metrics_data[:],
        names=set(m.name for m in metrics).union(set(md.name for md in metrics_data))
    )

T = typing.TypeVar("T")

def _mock_get_metrics_proto(*,
                            message_type: type[T],
                            session: requests.Session,
                            url: str) -> str:
    assert session is not None
    if message_type is MockMetric:
        return _METRICS_URL_TO_MESSAGE_MAP[url]
    if message_type is MockMetricsData:
        return _METRICS_DATA_URL_TO_MESSAGE_MAP[url]
    raise RuntimeError("Bad type encountered!")


@patch("resim.metrics.fetch_job_metrics.fetch_metrics_urls.fetch_metrics_urls",
       new=_mock_fetch_metrics_urls)
@patch("resim.metrics.fetch_job_metrics.fetch_metrics_urls.fetch_metrics_data_urls",
       new=_mock_fetch_metrics_data_urls)
@patch("resim.metrics.fetch_job_metrics.get_metrics_proto",
       new=_mock_get_metrics_proto)
@patch("resim.metrics.fetch_job_metrics.mp.Metric", MockMetric)
@patch("resim.metrics.fetch_job_metrics.mp.MetricsData", MockMetricsData)
class FetchJobMetricsTest(unittest.TestCase):
    """
    The unit test case itself.
    """

    def test_fetch_job_metrics(self) -> None:
        """
        Test that we can fetch job metrics while mocking the metrics url and
        proto getters.
        """

        _FETCHED_JOB_IDS = [
            uuid.UUID("1f2441f6-cee2-4078-9729-048810c1da96"),
            uuid.UUID("f1b4fa78-7ede-46c8-bd8e-deb5681e8010"),
        ]

        metrics_protos, metrics_data_protos = fjm.fetch_job_metrics(
            token=_TOKEN,
            base_url="https://api.resim.ai/v1",
            jobs=[fjm.JobInfo(
                job_id=uuid.UUID("1f2441f6-cee2-4078-9729-048810c1da96"),
                batch_id=uuid.UUID("b6ff9ce1-a481-4793-a9ea-d607f6efe628"),
            ),
                fjm.JobInfo(
                    job_id=uuid.UUID("f1b4fa78-7ede-46c8-bd8e-deb5681e8010"),
                    batch_id=uuid.UUID("0a74e080-7782-4a6c-ac26-248caa575405"),
            )])

        # Check that the job ids are correct and present
        self.assertEqual(len(_FETCHED_JOB_IDS), len(metrics_protos))
        self.assertEqual(len(_FETCHED_JOB_IDS), len(metrics_data_protos))
        for job_id in _FETCHED_JOB_IDS:
            metric_urls =_JOB_ID_METRICS_URL_MAP[job_id]
            metrics_data_urls = _JOB_ID_METRICS_DATA_URL_MAP[job_id]
            self.assertIn(job_id, metrics_protos)
            self.assertIn(job_id, metrics_data_protos)

            # Check that the correct data are present
            self.assertEqual(len(metrics_protos[job_id]), len(metric_urls))
            self.assertEqual(
                len(metrics_data_protos[job_id]), len(metrics_data_urls))
            for url in metric_urls:
                self.assertIn(
                    _METRICS_URL_TO_MESSAGE_MAP[url],
                    metrics_protos[job_id])
            for url in metrics_data_urls:
                self.assertIn(
                    _METRICS_DATA_URL_TO_MESSAGE_MAP[url],
                    metrics_data_protos[job_id])

@patch("resim.metrics.fetch_job_metrics.fetch_metrics_urls.fetch_metrics_urls",
       new=_mock_fetch_metrics_urls)
@patch("resim.metrics.fetch_job_metrics.fetch_metrics_urls.fetch_metrics_data_urls",
       new=_mock_fetch_metrics_data_urls)
@patch("resim.metrics.fetch_job_metrics.get_metrics_proto",
       new=_mock_get_metrics_proto)
@patch("resim.metrics.fetch_job_metrics.unpack_metrics",
       new=_mock_unpack_metrics)
@patch("resim_python_client.api.batches.list_jobs.sync",
       new=_mock_list_job_ids_by_batch)
@patch("resim_python_client.models.list_jobs_response_200.ListJobsResponse200",
       MockListJobsResponse200)
@patch("resim.metrics.fetch_job_metrics.UnpackedMetrics", MockUnpackedMetrics)
@patch("resim_python_client.models.job.Job", MockJob)
@patch("resim.metrics.fetch_job_metrics.mp.Metric", MockMetric)
@patch("resim.metrics.fetch_job_metrics.mp.MetricsData", MockMetricsData)
class FetchJobMetricsByBatchTest(unittest.TestCase):
    """
    The batch test case itself.
    """

    def test_fetch_job_metrics_by_batch(self) -> None:
        """
        Test that we can fetch job metrics while mocking the metrics url and
        proto getters.
        """
        for batch_id in _BATCH_IDS:
            job_to_metrics = fjm.fetch_job_metrics_by_batch(
                token=_TOKEN,
                api_url="https://api.resim.ai/v1",
                batch_id=batch_id
            )

            self.assertEqual(len(job_to_metrics), len(_BATCH_IDS_TO_JOB_IDS_MAP[batch_id]))
            self.assertEqual(set(job_to_metrics.keys()), set(_BATCH_IDS_TO_JOB_IDS_MAP[batch_id]))

            for job_id, metrics in job_to_metrics.items():
                self.assertEqual(
                    set(metrics.metrics),
                    set(_METRICS_URL_TO_MESSAGE_MAP[url]
                        for url in _JOB_ID_METRICS_URL_MAP[job_id]))
                self.assertEqual(
                    set(metrics.metrics_data),
                    set(_METRICS_DATA_URL_TO_MESSAGE_MAP[url]
                        for url in _JOB_ID_METRICS_DATA_URL_MAP[job_id]))

                expected_names = (
                    set(_METRICS_URL_TO_MESSAGE_MAP[url].name
                        for url in _JOB_ID_METRICS_URL_MAP[job_id])
                    .union(set(_METRICS_DATA_URL_TO_MESSAGE_MAP[url].name
                               for url in _JOB_ID_METRICS_DATA_URL_MAP[job_id]))
                )
                self.assertEqual(metrics.names, expected_names)

if __name__ == '__main__':
    unittest.main()
