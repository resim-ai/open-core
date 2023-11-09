# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Tests for fetch_metrics_urls.py
"""

import uuid
import unittest
from dataclasses import dataclass
import typing
from unittest.mock import patch

from resim_python_client.client import AuthenticatedClient
from resim_python_client.api.batches import (
    list_metrics_for_job, list_metrics_data_for_job)

from resim.metrics import fetch_metrics_urls


@dataclass
class MockMetric:
    """A mock for the metric-url-containing part of the metrics response."""
    metric_url: str


@dataclass
class MockMetricsResponse:
    """A mock for the response the list_metrics_for_job endpoint"""
    metrics: list[MockMetric]


@dataclass
class MockMetricsData:
    """
    A mock for the metrics-data-url-containing part of the metrics data
    response.
    """
    metrics_data_url: str


@dataclass
class MockMetricsDataResponse:
    """A mock for the response the list_metrics_data_for_job endpoint"""
    metrics_data: list[MockMetricsData]


@patch("resim_python_client.client.AuthenticatedClient")
class FetchMetricsUrlsTest(unittest.TestCase):
    """The unit test case itself."""

    def test_fetch_metrics_urls(
            self,
            authenticated_client_mock: unittest.mock.MagicMock) -> None:
        """
        Test that fetch_metrics_urls() works with a mocked version of
        fetch_all_pages().
        """
        test_batch_id = uuid.uuid4()
        test_job_id = uuid.uuid4()
        test_client = authenticated_client_mock()

        def mock_fetch_all_pages(
                endpoint: typing.Callable,
                batch_id: str,
                job_id: str,
                *,
                client: AuthenticatedClient) -> list[MockMetricsResponse]:
            """A mock for the fetch_all_pages() function that we use to return mock metrics"""
            self.assertEqual(endpoint, list_metrics_for_job.sync)
            self.assertEqual(uuid.UUID(batch_id), test_batch_id)
            self.assertEqual(uuid.UUID(job_id), test_job_id)
            self.assertEqual(client, test_client)
            return [
                MockMetricsResponse(
                    metrics=[
                        MockMetric(
                            metric_url=f"https://www.test_url.com/metrics_{i}_{j}.binproto",
                        ) for i in range(3)]) for j in range(3)]

        with patch("resim.metrics.fetch_metrics_urls.fetch_all_pages",
                   new=mock_fetch_all_pages) as _:
            metrics_urls = fetch_metrics_urls.fetch_metrics_urls(
                batch_id=test_batch_id,
                job_id=test_job_id,
                client=test_client)
            self.assertEqual(
                metrics_urls, [
                    f"https://www.test_url.com/metrics_{i}_{j}.binproto"
                    for j in range(3) for i in range(3)])

    def test_fetch_metrics_data_urls(
            self, authenticated_client_mock: unittest.mock.MagicMock) -> None:
        """
        Test that fetch_metrics_data_urls() works with a mocked version of
        fetch_all_pages().
        """
        test_batch_id = uuid.uuid4()
        test_job_id = uuid.uuid4()
        test_client = authenticated_client_mock()

        def mock_fetch_all_pages(
                endpoint: typing.Callable,
                batch_id: str,
                job_id: str,
                *,
                client: AuthenticatedClient) -> list[MockMetricsDataResponse]:
            """A mock for the fetch_all_pages() function that we use to return mock metrics data"""
            self.assertEqual(endpoint, list_metrics_data_for_job.sync)
            self.assertEqual(uuid.UUID(batch_id), test_batch_id)
            self.assertEqual(uuid.UUID(job_id), test_job_id)
            self.assertEqual(client, test_client)
            return [
                MockMetricsDataResponse(
                    metrics_data=[
                        MockMetricsData(
                            metrics_data_url=f"https://www.test_url.com/metrics_{i}_{j}.binproto",
                        ) for i in range(3)]) for j in range(3)]

        with patch("resim.metrics.fetch_metrics_urls.fetch_all_pages",
                   new=mock_fetch_all_pages) as _:
            metrics_data_urls = fetch_metrics_urls.fetch_metrics_data_urls(
                batch_id=test_batch_id,
                job_id=test_job_id,
                client=test_client)
            self.assertEqual(
                metrics_data_urls, [
                    f"https://www.test_url.com/metrics_{i}_{j}.binproto"
                    for j in range(3) for i in range(3)])


if __name__ == "__main__":
    unittest.main()
