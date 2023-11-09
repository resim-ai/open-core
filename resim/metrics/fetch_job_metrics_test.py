
import unittest
import uuid

from dataclasses import dataclass

import requests
import typing
from unittest.mock import patch
from resim_python_client.resim_python_client.client import AuthenticatedClient
import resim.metrics.fetch_job_metrics as fjm
import resim.auth.python.device_code_client as dcc


def _get_token() -> str:
    client = dcc.DeviceCodeClient(domain="https://resim.us.auth0.com")
    token: str = client.get_jwt()["access_token"]
    return token


_TOKEN = "wowwhatacoincidentaltoken"


@dataclass
class MockMetric:
    data: str


@dataclass
class MockMetricsData:
    data: str


_JOB_ID_METRICS_URL_MAP = {
    uuid.UUID("1f2441f6-cee2-4078-9729-048810c1da96"):
        [f"https://example.com/metrics_{i}.binproto" for i in range(3)],
    uuid.UUID("f1b4fa78-7ede-46c8-bd8e-deb5681e8010"):
        [f"https://example.com/metrics_{i}.binproto" for i in range(3, 6)],
}

_JOB_ID_METRICS_DATA_URL_MAP = {
    uuid.UUID("1f2441f6-cee2-4078-9729-048810c1da96"):
        [f"https://example.com/metrics_data_{i}.binproto" for i in range(3)],
    uuid.UUID("f1b4fa78-7ede-46c8-bd8e-deb5681e8010"):
        [f"https://example.com/metrics_data_{i}.binproto" for i in range(3, 6)],
}

_METRICS_URL_TO_MESSAGE_MAP = {
    f"https://example.com/metrics_{i}.binproto": f"metric_{i}" for i in range(6)
}

_METRICS_DATA_URL_TO_MESSAGE_MAP = {
    f"https://example.com/metrics_data_{i}.binproto": f"data_{i}" for i in range(6)
}


def _mock_fetch_metrics_urls(*,
                             batch_id: uuid.UUID,
                             job_id: uuid.UUID,
                             client: AuthenticatedClient) -> list[str]:
    assert client.token == _TOKEN
    return list(_JOB_ID_METRICS_URL_MAP[job_id])


def _mock_fetch_metrics_data_urls(*,
                                  batch_id: uuid.UUID,
                                  job_id: uuid.UUID,
                                  client: AuthenticatedClient) -> list[str]:
    assert client.token == _TOKEN
    return list(_JOB_ID_METRICS_DATA_URL_MAP[job_id])


T = typing.TypeVar("T")


def _mock_get_metrics_proto(*,
                            message_type: type[T],
                            session: requests.Session,
                            url: str) -> str:
    if message_type is MockMetric:
        return _METRICS_URL_TO_MESSAGE_MAP[url]
    elif message_type is MockMetricsData:
        return _METRICS_DATA_URL_TO_MESSAGE_MAP[url]
    else:
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
    def test_fetch_job_metrics(self) -> None:
        metrics_protos, metrics_data_protos = fjm.fetch_job_metrics(
            token=_TOKEN,
            jobs=[fjm.JobInfo(
                job_id=uuid.UUID("1f2441f6-cee2-4078-9729-048810c1da96"),
                batch_id=uuid.UUID("b6ff9ce1-a481-4793-a9ea-d607f6efe628"),
            ),
                fjm.JobInfo(
                    job_id=uuid.UUID("f1b4fa78-7ede-46c8-bd8e-deb5681e8010"),
                    batch_id=uuid.UUID("0a74e080-7782-4a6c-ac26-248caa575405"),
            )])

        # Check that the job ids are correct and present
        self.assertEqual(len(_JOB_ID_METRICS_URL_MAP), len(metrics_protos))
        self.assertEqual(
            len(_JOB_ID_METRICS_URL_MAP),
            len(metrics_data_protos))
        for job_id, metric_urls in _JOB_ID_METRICS_URL_MAP.items():
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


if __name__ == '__main__':
    unittest.main()
