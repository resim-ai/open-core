

"""
This module contains functions for fetching a set of job metrics from resim's API.
"""

import uuid
import collections
from http import HTTPStatus
import threading
from typing import Any, Callable
from dataclasses import dataclass

import requests
from resim_python_client.resim_python_client.client import AuthenticatedClient
from resim_python_client.resim_python_client.api.batches import (
    list_jobs, list_metrics_for_job, list_metrics_data_for_job)

import resim.metrics.proto.metrics_pb2 as mp
import resim.auth.python.device_code_client as dcc
from resim.metrics import fetch_all_pages

from resim.metrics import fetch_metrics_urls
from resim.metrics.get_metrics_proto import get_metrics_proto


def _get_metrics_urls(*,
                      batch_id: uuid.UUID,
                      job_id: uuid.UUID,
                      client: AuthenticatedClient,
                      metrics_urls: dict[uuid.UUID, list[str]],
                      metrics_urls_lock: threading.Lock) -> None:
    metrics_urls_lock.acquire()
    metrics_urls[job_id] = fetch_metrics_urls.fetch_metrics_urls(
        batch_id=batch_id, job_id=job_id, client=client)
    metrics_urls_lock.release()


def _get_metrics_data_urls(*,
                           batch_id: uuid.UUID,
                           job_id: uuid.UUID,
                           client: AuthenticatedClient,
                           metrics_data_urls: dict[uuid.UUID, list[str]],
                           metrics_data_urls_lock: threading.Lock) -> None:
    metrics_data_urls_lock.acquire()
    metrics_data_urls[job_id] = fetch_metrics_urls.fetch_metrics_data_urls(
        batch_id=batch_id, job_id=job_id, client=client)
    metrics_data_urls_lock.release()


def _fetch_metrics(*,
                   message_type: type[Any],
                   session: requests.Session,
                   job_id: uuid.UUID,
                   url: str,
                   protos: dict[uuid.UUID, list[Any]],
                   protos_lock: threading.Lock) -> None:
    protos_lock.acquire()
    protos[job_id].append(
        get_metrics_proto(
            message_type=message_type,
            session=session,
            url=url))
    protos_lock.release()


@dataclass
class JobInfo:
    job_id: uuid.UUID
    batch_id: uuid.UUID


def fetch_job_metrics(*,
                      token: str,
                      jobs: list[JobInfo]) -> tuple[collections.defaultdict[uuid.UUID,
                                                                            mp.Metric],
                                                    collections.defaultdict[uuid.UUID,
                                                                            mp.MetricsData]]:
    """
    This function fetches all job metrics from job_ids whose batch ids are in batch_ids.
    """

    client = AuthenticatedClient(
        base_url="https://api.resim.ai/v1",
        token=token)

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
                    "batch_id": job.batch_id,
                    "job_id": job.job_id,
                    "client": client,
                    "metrics_urls": metrics_urls,
                    "metrics_urls_lock": metrics_urls_lock}))
        threads.append(
            threading.Thread(
                target=_get_metrics_data_urls,
                kwargs={
                    "batch_id": job.batch_id,
                    "job_id": job.job_id,
                    "client": client,
                    "metrics_data_urls": metrics_data_urls,
                    "metrics_data_urls_lock": metrics_data_urls_lock}))

    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    threads = []

    metrics_protos: collections.defaultdict[uuid.UUID, mp.Metric] = (
        collections.defaultdict(lambda: []))
    metrics_protos_lock = threading.Lock()

    metrics_data_protos: collections.defaultdict[uuid.UUID, mp.MetricsData] = (
        collections.defaultdict(lambda: []))
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
                        "protos_lock": metrics_protos_lock}))

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
                        "protos_lock": metrics_data_protos_lock}))

    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    session.close()
    client.get_httpx_client().close()

    return metrics_protos, metrics_data_protos
