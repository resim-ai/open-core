# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Tests for fetch_metrics_urls.py
"""

import unittest
from dataclasses import dataclass
from http import HTTPStatus
import typing
import resim.metrics.fetch_metrics_urls as fetch_metrics_urls
from resim_python_client.resim_python_client.client import AuthenticatedClient
import uuid
from unittest.mock import patch
from resim_python_client.resim_python_client.api.batches import (
    list_metrics_for_job, list_metrics_data_for_job)


@dataclass
class MockMetric:
    metric_url: str


@dataclass
class MockMetricsResponse:
    metrics: list[MockMetric]


@dataclass
class MockMetricsData:
    metrics_data_url: str


@dataclass
class MockMetricsDataResponse:
    metrics_data: list[MockMetricsData]


@patch("resim_python_client.resim_python_client.client.AuthenticatedClient")
class FetchMetricsUrlsTest(unittest.TestCase):
    def test_fetch_metrics_urls(self, AuthenticatedClientMock: unittest.mock.MagicMock) -> None:
        test_batch_id = uuid.uuid4()
        test_job_id = uuid.uuid4()
        def mock_fetch_all_pages(
                endpoint: typing.Callable,
                batch_id: str,
                job_id: str,
                *,
                client: AuthenticatedClient) -> list[MockMetricsResponse]:
            self.assertEqual(endpoint, list_metrics_for_job.sync)
            self.assertEqual(uuid.UUID(batch_id), test_batch_id)
            self.assertEqual(uuid.UUID(job_id), test_job_id)
            return [
                MockMetricsResponse(
                    metrics=[
                        MockMetric(
                            metric_url=f"https://www.test_url.com/metrics_{i}_{j}.binproto",
                        ) for i in range(3)]) for j in range(3)]
        

        with patch("resim.metrics.fetch_metrics_urls.fetch_all_pages",
                   new=mock_fetch_all_pages) as p:
            client = AuthenticatedClientMock()
            metrics_urls = fetch_metrics_urls.fetch_metrics_urls(
                batch_id=test_batch_id,
                job_id=test_job_id,
                client=client)
            self.assertEqual(
                metrics_urls, [
                    f"https://www.test_url.com/metrics_{i}_{j}.binproto"
                    for j in range(3) for i in range(3)])
            
    def test_fetch_metrics_data_urls(self, AuthenticatedClientMock: unittest.mock.MagicMock) -> None:
        test_batch_id = uuid.uuid4()
        test_job_id = uuid.uuid4()
        def mock_fetch_all_pages(
                endpoint: typing.Callable,
                batch_id: str,
                job_id: str,
                *,
                client: AuthenticatedClient) -> list[MockMetricsDataResponse]:
            self.assertEqual(endpoint, list_metrics_data_for_job.sync)
            self.assertEqual(uuid.UUID(batch_id), test_batch_id)
            self.assertEqual(uuid.UUID(job_id), test_job_id)
            return [
                MockMetricsDataResponse(
                    metrics_data=[
                        MockMetricsData(
                            metrics_data_url=f"https://www.test_url.com/metrics_{i}_{j}.binproto",
                        ) for i in range(3)]) for j in range(3)]
        
        with patch("resim.metrics.fetch_metrics_urls.fetch_all_pages",
                   new=mock_fetch_all_pages) as p:
            client = AuthenticatedClientMock()
            metrics_data_urls = fetch_metrics_urls.fetch_metrics_data_urls(
                batch_id=test_batch_id,
                job_id=test_job_id,
                client=client)
            self.assertEqual(
                metrics_data_urls, [
                    f"https://www.test_url.com/metrics_{i}_{j}.binproto"
                    for j in range(3) for i in range(3)])


if __name__ == "__main__":
    unittest.main()
