# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
This module contains functions for querying the presigned urls of metrics and
metrics data.
"""

import uuid

from resim_python_client.api.batches import (
    list_metrics_data_for_job,
    list_metrics_for_job,
)
from resim_python_client.client import AuthenticatedClient
from resim_python_client.models.metrics_data import MetricsData
from resim_python_client.models.metrics_data_type import MetricsDataType
from resim_python_client.models import ListJobMetricsOutput

from resim.metrics.fetch_all_pages import fetch_all_pages


def fetch_metrics_urls(
    *,
    project_id: uuid.UUID,
    batch_id: uuid.UUID,
    job_id: uuid.UUID,
    client: AuthenticatedClient,
) -> list[str]:
    """Fetch all metrics urls for a given job_id."""
    responses: list[ListJobMetricsOutput] = fetch_all_pages(
        list_metrics_for_job.sync,
        str(project_id),
        str(batch_id),
        str(job_id),
        client=client,
    )

    return [
        metric.metric_url
        for metrics_response in responses
        for metric in metrics_response.metrics
    ]


def fetch_metrics_data_urls(
    *,
    project_id: uuid.UUID,
    batch_id: uuid.UUID,
    job_id: uuid.UUID,
    client: AuthenticatedClient,
) -> list[str]:
    """Fetch all metrics data urls for a given job_id."""

    def is_standard(data: MetricsData) -> bool:
        result: bool = data.metrics_data_type == MetricsDataType.STANDARD
        return result

    responses: list[ListJobMetricsOutput] = fetch_all_pages(
        list_metrics_data_for_job.sync,
        str(project_id),
        str(batch_id),
        str(job_id),
        client=client,
    )
    return [
        metrics_data.metrics_data_url
        for metrics_data_response in responses
        for metrics_data in metrics_data_response.metrics_data
        if is_standard(metrics_data)
    ]
