

import uuid
import collections
import requests
from http import HTTPStatus
import threading

import resim.metrics.proto.metrics_pb2 as mp
import resim.auth.python.device_code_client as dcc

from resim_python_client.resim_python_client.client import AuthenticatedClient
import resim_python_client.resim_python_client.api.batches.list_jobs as list_jobs
import resim_python_client.resim_python_client.api.batches.list_metrics_for_job as list_metrics_for_job
import resim_python_client.resim_python_client.api.batches.list_metrics_data_for_job as list_metrics_data_for_job


def _get_token():
    client = dcc.DeviceCodeClient(domain="https://resim.us.auth0.com")
    return client.get_jwt()["access_token"]


def _fetch_single_job_metrics(metrics_map,
                              metrics_map_lock,
                              *,
                              thread_list,
                              thread_list_lock,
                              client: AuthenticatedClient,
                              batch_id: uuid.UUID,
                              job_id: uuid.UUID):
    metrics_response = list_metrics_for_job.sync(
        str(batch_id), str(job_id), client=client)
    for metric in metrics_response.metrics:

        metric_message_response = requests.get(metric.metric_url)
        assert metric_message_response.status_code == HTTPStatus.OK
        print(metric_message_response.content)


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
                   session: requests.Session,
                   job_id: uuid.UUID,
                   metrics_url: str,
                   metrics_protos,
                   metrics_protos_lock):
    response = session.get(metrics_url)
    assert response.status_code == HTTPStatus.OK
    metrics_protos_lock.acquire()
    metric = mp.Metric()
    metric.ParseFromString(response.content)
    metrics_protos[job_id].append(metric)
    metrics_protos_lock.release()

def _fetch_metrics_data(*,
                   session: requests.Session,                        
                   job_id: uuid.UUID,
                   metrics_data_url: str,
                   metrics_data_protos,
                   metrics_data_protos_lock):
    response = session.get(metrics_data_url)
    assert response.status_code == HTTPStatus.OK
    metrics_data_protos_lock.acquire()
    metrics_data = mp.MetricsData()
    metrics_data.ParseFromString(response.content)
    metrics_data_protos[job_id].append(metrics_data)
    metrics_data_protos_lock.release()
    


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
                        "session": session,
                        "job_id": job_id,
                        "metrics_url": url,
                        "metrics_protos": metrics_protos,
                        "metrics_protos_lock": metrics_protos_lock}))
    for job_id, urllist in metrics_data_urls.items():
        for url in urllist:
            threads.append(
                threading.Thread(
                target=_fetch_metrics_data,
                kwargs={
                    "session": session,                    
                    "job_id": job_id,
                    "metrics_data_url": url,
                    "metrics_data_protos": metrics_data_protos,
                    "metrics_data_protos_lock": metrics_data_protos_lock}))
            
            
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    session.close()

    return metrics_protos, metrics_data_protos
