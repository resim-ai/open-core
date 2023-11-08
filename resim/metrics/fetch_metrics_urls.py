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
from resim_python_client.resim_python_client.client import AuthenticatedClient
from resim.metrics.fetch_all_pages import fetch_all_pages
from resim_python_client.resim_python_client.api.batches import (
    list_metrics_for_job, list_metrics_data_for_job)


def fetch_metrics_urls(*,
                       batch_id: uuid.UUID,
                       job_id: uuid.UUID,
                       client: AuthenticatedClient) -> list[str]:
    return [
        metric.metric_url for metrics_response in fetch_all_pages(
            list_metrics_for_job.sync,
            str(batch_id),
            str(job_id),
            client=client) for metric in metrics_response.metrics]


def fetch_metrics_data_urls(*,
                            batch_id: uuid.UUID,
                            job_id: uuid.UUID,
                            client: AuthenticatedClient) -> list[str]:
    return [
        metrics_data.metrics_data_url for metrics_data_response in fetch_all_pages(
            list_metrics_data_for_job.sync,
            str(batch_id),
            str(job_id),
            client=client) for metrics_data in metrics_data_response.metrics_data]
