

import uuid
import collections
import requests
from http import HTTPStatus
import threading
from typing import Any

import resim.metrics.proto.metrics_pb2 as mp
import resim.auth.python.device_code_client as dcc

from resim_python_client.resim_python_client.client import AuthenticatedClient
import resim_python_client.resim_python_client.api.batches.list_jobs as list_jobs
import resim_python_client.resim_python_client.api.batches.list_metrics_for_job as list_metrics_for_job
import resim_python_client.resim_python_client.api.batches.list_metrics_data_for_job as list_metrics_data_for_job


def _get_token():
    client = dcc.DeviceCodeClient(domain="https://resim.us.auth0.com")
    return client.get_jwt()["access_token"]


def _get_metrics_urls(*,
                      batch_id: uuid.UUID,
                      job_id: uuid.UUID,
                      client: AuthenticatedClient,
                      metrics_urls,
                      metrics_urls_lock):
    metrics_response = list_metrics_for_job.sync(
        str(batch_id), str(job_id), client=client)
    metrics_urls_lock.acquire()
    metrics_urls[job_id] = [metric.metric_url for metric in metrics_response.metrics]
    metrics_urls_lock.release()

def _get_metrics_data_urls(*,
                      batch_id: uuid.UUID,
                      job_id: uuid.UUID,
                      client: AuthenticatedClient,
                      metrics_data_urls,
                      metrics_data_urls_lock):
    metrics_data_response = list_metrics_data_for_job.sync(
        str(batch_id), str(job_id), client=client)
    metrics_data_urls_lock.acquire()
    metrics_data_urls[job_id] = [metrics_data.metrics_data_url for metrics_data in metrics_data_response.metrics_data]
    metrics_data_urls_lock.release()
    


def _fetch_metrics(*,
                   message_type: type[Any],
                   session: requests.Session,
                   job_id: uuid.UUID,
                   url: str,
                   protos: list[Any],
                   protos_lock: threading.Lock):
    response = session.get(url)
    assert response.status_code == HTTPStatus.OK
    protos_lock.acquire()
    message = message_type()
    message.ParseFromString(response.content)
    protos[job_id].append(message)
    protos_lock.release()



def fetch_job_metrics(*, batch_ids: list[uuid.UUID], job_ids: list[uuid.UUID]):
    token = _get_token()

    client = AuthenticatedClient(
        base_url="https://api.resim.ai/v1",
        token=token)

    job_ids_set = set(job_ids)

    metrics_urls = {}
    metrics_urls_lock = threading.Lock()

    metrics_data_urls = {}
    metrics_data_urls_lock = threading.Lock()
    
    threads = []
    for batch_id in batch_ids:
        response = list_jobs.sync(batch_id, client=client)
        for job in response.jobs:
            job_id = uuid.UUID(job.job_id)
            if job_id in job_ids_set:
                threads.append(
                    threading.Thread(
                        target=_get_metrics_urls,
                        kwargs={
                            "batch_id": batch_id,
                            "job_id": job_id,
                            "client": client,
                            "metrics_urls": metrics_urls,
                            "metrics_urls_lock": metrics_urls_lock}))
                threads.append(
                    threading.Thread(
                        target=_get_metrics_data_urls,
                        kwargs={
                            "batch_id": batch_id,
                            "job_id": job_id,
                            "client": client,
                            "metrics_data_urls": metrics_data_urls,
                            "metrics_data_urls_lock": metrics_data_urls_lock}))
                

    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    threads = []

    metrics_protos = collections.defaultdict(lambda: [])
    metrics_protos_lock = threading.Lock()
    
    metrics_data_protos = collections.defaultdict(lambda: [])
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
