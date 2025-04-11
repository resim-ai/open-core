# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
This module contains functions for fetching a set of job metrics from resim's API.
"""

import collections
import threading
import uuid
from dataclasses import dataclass
from typing import Any, Dict

import requests
from resim_python_client.api.batches import list_jobs
from resim_python_client.models import Job
from resim_python_client.client import AuthenticatedClient

import resim.metrics.proto.metrics_pb2 as mp
from resim.metrics import fetch_metrics_urls
from resim.metrics.fetch_all_pages import fetch_all_pages
from resim.metrics.get_metrics_proto import get_metrics_proto
from resim.metrics.python.unpack_metrics import UnpackedMetrics, unpack_metrics


@dataclass
class JobInfo:
    """A simple class representing the job_id, batch_id, and project_id for a job."""

    job_id: uuid.UUID
    batch_id: uuid.UUID
    project_id: uuid.UUID


def fetch_job_metrics_by_batch(
    *, token: str, api_url: str, project_id: uuid.UUID, batch_id: uuid.UUID
) -> Dict[uuid.UUID, UnpackedMetrics]:
    """
    This downloads all metrics associated with a certain batch, and stores them in a
    dictionary mapping each job ID to the metrics and metrics data associated with
    those job IDs.
    """

    # Fetch the jobs for this batch
    client = AuthenticatedClient(base_url=api_url, token=token)
    jobs_responses: list[Job] = fetch_all_pages(
        list_jobs.sync, str(project_id), str(batch_id), client=client
    )
    jobs = [
        JobInfo(job_id=uuid.UUID(job.job_id), batch_id=batch_id, project_id=project_id)
        for jobs_response in jobs_responses
        for job in jobs_response.jobs
    ]

    # Fetch the metrics for these jobs:
    metrics_protos, metrics_data_protos = fetch_job_metrics(
        token=token, base_url=api_url, jobs=jobs
    )

    # Unpack the fetched metrics
    unpacked_metrics_per_job = {}
    for job in jobs:
        metrics = metrics_protos[job.job_id]
        metrics_data = metrics_data_protos[job.job_id]
        unpacked_metrics_per_job[job.job_id] = unpack_metrics(
            metrics=metrics,
            metrics_data=metrics_data,
            events=[],
        )

    return unpacked_metrics_per_job


def fetch_job_metrics(
    *, token: str, base_url: str, jobs: list[JobInfo]
) -> tuple[dict[uuid.UUID, list[mp.Metric]], dict[uuid.UUID, list[mp.MetricsData]]]:
    """
    This function fetches all job metrics from all specified jobs.
    """

    # Step 1: Fetch all the presigned urls per job
    client = AuthenticatedClient(base_url=base_url, token=token)

    metrics_urls, metrics_data_urls = _fetch_all_urls(client=client, jobs=jobs)

    client.get_httpx_client().close()

    # Step 2: Fetch all of the protos per job
    metrics_protos, metrics_data_protos = _fetch_all_protos(
        metrics_urls=metrics_urls, metrics_data_urls=metrics_data_urls
    )

    return metrics_protos, metrics_data_protos


def _fetch_all_urls(
    *, client: AuthenticatedClient, jobs: list[JobInfo]
) -> tuple[dict[uuid.UUID, list[str]], dict[uuid.UUID, list[str]]]:
    """
    Fetch all metrics and metrics data urls

    This helper fetches all of the urls for the metrics and metrics
    data objects in each job and places them into a dictionary keyed
    by the job ids.
    """
    metrics_urls: dict[uuid.UUID, list[str]] = {}
    metrics_urls_lock = threading.Lock()

    metrics_data_urls: dict[uuid.UUID, list[str]] = {}
    metrics_data_urls_lock = threading.Lock()

    threads = []
    for job in jobs:
        threads.append(
            threading.Thread(
                target=_get_metrics_urls,
                kwargs={
                    "project_id": job.project_id,
                    "batch_id": job.batch_id,
                    "job_id": job.job_id,
                    "client": client,
                    "metrics_urls": metrics_urls,
                    "metrics_urls_lock": metrics_urls_lock,
                },
            )
        )
        threads.append(
            threading.Thread(
                target=_get_metrics_data_urls,
                kwargs={
                    "project_id": job.project_id,
                    "batch_id": job.batch_id,
                    "job_id": job.job_id,
                    "client": client,
                    "metrics_data_urls": metrics_data_urls,
                    "metrics_data_urls_lock": metrics_data_urls_lock,
                },
            )
        )

    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    return metrics_urls, metrics_data_urls


def _fetch_all_protos(
    *,
    metrics_urls: dict[uuid.UUID, list[str]],
    metrics_data_urls: dict[uuid.UUID, list[str]],
) -> tuple[dict[uuid.UUID, list[mp.Metric]], dict[uuid.UUID, list[mp.MetricsData]]]:
    """
    Get and parse the protobuf data from the given urls

    This helper reads in dictionaries of job_id -> metrics_proto_url
    for both metrics and metrics data, fetches from the given urls,
    and deserializes the content into the appropriate protobuf metrics
    types.
    """
    threads = []

    metrics_protos: collections.defaultdict[uuid.UUID, list[mp.Metric]] = (
        collections.defaultdict(lambda: [])
    )
    metrics_protos_lock = threading.Lock()

    metrics_data_protos: collections.defaultdict[uuid.UUID, list[mp.MetricsData]] = (
        collections.defaultdict(lambda: [])
    )
    metrics_data_protos_lock = threading.Lock()

    session = requests.Session()
    for job_id, urllist in metrics_urls.items():
        for url in urllist:
            threads.append(
                threading.Thread(
                    target=_fetch_metrics,
                    kwargs={
                        "message_type": mp.Metric,
                        "session": session,
                        "job_id": job_id,
                        "url": url,
                        "protos": metrics_protos,
                        "protos_lock": metrics_protos_lock,
                    },
                )
            )

    for job_id, urllist in metrics_data_urls.items():
        for url in urllist:
            threads.append(
                threading.Thread(
                    target=_fetch_metrics,
                    kwargs={
                        "message_type": mp.MetricsData,
                        "session": session,
                        "job_id": job_id,
                        "url": url,
                        "protos": metrics_data_protos,
                        "protos_lock": metrics_data_protos_lock,
                    },
                )
            )

    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    session.close()

    return metrics_protos, metrics_data_protos


def _get_metrics_urls(
    *,
    project_id: uuid.UUID,
    batch_id: uuid.UUID,
    job_id: uuid.UUID,
    client: AuthenticatedClient,
    metrics_urls: dict[uuid.UUID, list[str]],
    metrics_urls_lock: threading.Lock,
) -> None:
    """
    Simple wrapper around fetch_metrics_urls which locks a mutex
    so we can multithread this.
    """
    metrics_urls_lock.acquire()
    metrics_urls[job_id] = fetch_metrics_urls.fetch_metrics_urls(
        project_id=project_id, batch_id=batch_id, job_id=job_id, client=client
    )
    metrics_urls_lock.release()


def _get_metrics_data_urls(
    *,
    project_id: uuid.UUID,
    batch_id: uuid.UUID,
    job_id: uuid.UUID,
    client: AuthenticatedClient,
    metrics_data_urls: dict[uuid.UUID, list[str]],
    metrics_data_urls_lock: threading.Lock,
) -> None:
    """
    Simple wrapper around fetch_metrics_data_urls which locks a mutex
    so we can multithread this.
    """
    metrics_data_urls_lock.acquire()
    metrics_data_urls[job_id] = fetch_metrics_urls.fetch_metrics_data_urls(
        project_id=project_id, batch_id=batch_id, job_id=job_id, client=client
    )
    metrics_data_urls_lock.release()


def _fetch_metrics(
    *,
    message_type: type[Any],
    session: requests.Session,
    job_id: uuid.UUID,
    url: str,
    protos: dict[uuid.UUID, list[Any]],
    protos_lock: threading.Lock,
) -> None:
    """
    Simple wrapper around get_metrics_proto which locks a mutex
    so we can multithread this.
    """
    metrics_proto = get_metrics_proto(
        message_type=message_type, session=session, url=url
    )

    if (
        message_type == mp.Metric
        and metrics_proto.type == mp.MetricType.IMAGE_METRIC_TYPE
    ):
        return
    protos_lock.acquire()
    protos[job_id].append(metrics_proto)
    protos_lock.release()
