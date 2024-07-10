# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


import argparse
import asyncio
import itertools
import json
import logging
import pathlib
import uuid
from collections import defaultdict
from typing import Awaitable, Hashable

import httpx
import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from resim_python_client.api.batches import list_batch_metrics, list_metrics_for_job
from resim_python_client.client import AuthenticatedClient
from resim_python_client.models.batch import Batch
from resim_python_client.models.batch_metric import BatchMetric
from resim_python_client.models.batch_status import BatchStatus
from resim_python_client.models.job import Job
from resim_python_client.models.job_metric import JobMetric
from resim_python_client.models.job_status import JobStatus
from resim_python_client.models.metric_status import MetricStatus
from resim_python_client.models.metric_type import MetricType

import resim.metrics.python.metrics as rm
import resim.metrics.python.metrics_utils as mu
from resim.metrics.fetch_all_pages import async_fetch_all_pages
from resim.metrics.fetch_report_metrics import (
    fetch_batches_for_report,
    fetch_jobs_for_batches,
)
from resim.metrics.proto.metrics_pb2 import Metric as MetricProto
from resim.metrics.proto.validate_metrics_proto import validate_job_metrics
from resim.metrics.python.metrics_writer import ResimMetricsWriter

logger = logging.getLogger("resim")

DEFAULT_CONFIG_PATH = pathlib.Path("/tmp/resim/inputs/report_config.json")
DEFAULT_OUTPUT_PATH = pathlib.Path("/tmp/resim/outputs/metrics.binproto")


async def dict_gather(coroutine_dict: dict) -> dict:
    async def tag(key: Hashable, coroutine: Awaitable) -> tuple:
        return key, await coroutine

    return dict(
        await asyncio.gather(
            *(tag(key, coroutine) for (key, coroutine) in coroutine_dict.items())
        )
    )


async def main() -> None:
    logging.basicConfig()
    logger.setLevel(logging.INFO)

    parser = argparse.ArgumentParser(
        prog="default_report_metrics",
        description="Compute a basic set of report metrics.",
    )
    parser.add_argument("--report-config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--output-path", default=str(DEFAULT_OUTPUT_PATH))

    args = parser.parse_args()

    with open(args.report_config, "r", encoding="utf-8") as f:
        report_config = json.load(f)

    await compute_metrics(
        token=report_config["authToken"],
        api_url=report_config["apiURL"],
        project_id=uuid.UUID(report_config["projectID"]),
        report_id=uuid.UUID(report_config["reportID"]),
        output_path=args.output_path,
    )


async def _fetch_job_metrics_for_job(
    client: AuthenticatedClient, *, project_id: str, batch_id: str, job_id: str
) -> tuple[str, list[JobMetric]]:
    job_metrics = await async_fetch_all_pages(
        list_metrics_for_job.asyncio,
        project_id=project_id,
        batch_id=batch_id,
        job_id=job_id,
        client=client,
    )
    return job_id, [j for page in job_metrics for j in page.metrics]


async def _fetch_job_metrics_for_batch(
    client: AuthenticatedClient, *, project_id: str, batch_id: str, jobs: list[Job]
) -> list[tuple[str, list[JobMetric]]]:
    return list(
        await asyncio.gather(
            *[
                _fetch_job_metrics_for_job(
                    client, project_id=project_id, batch_id=batch_id, job_id=job.job_id
                )
                for job in jobs
            ]
        )
    )


async def fetch_job_metrics(
    client: AuthenticatedClient,
    *,
    project_id: uuid.UUID,
    batch_to_jobs_map: dict[str, list[Job]],
) -> dict[str, list[JobMetric]]:
    all_metrics = list(
        await asyncio.gather(
            *[
                _fetch_job_metrics_for_batch(
                    client, project_id=str(project_id), batch_id=batch_id, jobs=jobs
                )
                for (batch_id, jobs) in batch_to_jobs_map.items()
            ]
        )
    )

    return dict(itertools.chain.from_iterable(all_metrics))


async def _fetch_batch_metrics_for_batch(
    client: AuthenticatedClient, *, project_id: str, batch_id: str
) -> tuple[str, list[BatchMetric]]:
    batch_metrics = await async_fetch_all_pages(
        list_batch_metrics.asyncio,
        project_id=project_id,
        batch_id=batch_id,
        client=client,
    )
    return batch_id, [b for page in batch_metrics for b in page.batch_metrics]


async def fetch_batch_metrics(
    client: AuthenticatedClient,
    *,
    project_id: uuid.UUID,
    batch_ids: list[str],
) -> dict[str, list[BatchMetric]]:
    all_metrics = list(
        await asyncio.gather(
            *[
                _fetch_batch_metrics_for_batch(
                    client, project_id=str(project_id), batch_id=batch_id
                )
                for batch_id in batch_ids
            ]
        )
    )
    return dict(all_metrics)


async def fetch_scalar_batch_metrics(
    *, batch_to_metrics_map: dict[str, list[BatchMetric]]
) -> dict[str, dict[str, MetricProto]]:
    client = httpx.AsyncClient()
    tasks = {}

    async def unpack(query: Awaitable) -> MetricProto:
        response = await query
        message = MetricProto()
        message.ParseFromString(response.content)
        return message

    for batch, metrics in batch_to_metrics_map.items():
        batch_tasks = {}
        for metric in metrics:
            if metric.type == MetricType.SCALAR:
                batch_tasks[metric.name] = unpack(client.get(metric.metric_url))
        tasks[batch] = dict_gather(batch_tasks)
    responses = await dict_gather(tasks)
    return responses


async def compute_metrics(
    *,
    token: str,
    api_url: str,
    project_id: uuid.UUID,
    report_id: uuid.UUID,
    output_path: str,
) -> None:
    client = AuthenticatedClient(base_url=api_url, token=token)

    batches = await fetch_batches_for_report(
        client=client, report_id=report_id, project_id=project_id
    )
    logger.info("Fetched %d batches for report.", len(batches))

    batch_ids = [b.batch_id for b in batches]
    batch_to_jobs_map = await fetch_jobs_for_batches(
        client=client, batch_ids=batch_ids, project_id=project_id
    )

    for batch_id, jobs in batch_to_jobs_map.items():
        logger.info("Fetched %d jobs for batch %s", len(jobs), batch_id)

    job_to_metrics_map, batch_to_metrics_map = await asyncio.gather(
        fetch_job_metrics(
            client=client, project_id=project_id, batch_to_jobs_map=batch_to_jobs_map
        ),
        fetch_batch_metrics(client=client, project_id=project_id, batch_ids=batch_ids),
    )

    scalar_batch_metrics_map = await fetch_scalar_batch_metrics(
        batch_to_metrics_map=batch_to_metrics_map
    )

    logger.info(
        "Fetched metrics for %d jobs and %d batches",
        len(job_to_metrics_map),
        len(batch_to_metrics_map),
    )

    if len(batches) == 0:
        return

    writer = ResimMetricsWriter(uuid.uuid4())  # Make metrics writer!

    all_metrics = [
        job_status_categories_metric,
        flaky_experiences_metric,
        batch_metrics_scalars_over_time_metric,
    ]

    for metric in all_metrics:
        metric(
            writer,
            batches=batches,
            batch_to_jobs_map=batch_to_jobs_map,
            job_to_metrics_map=job_to_metrics_map,
            batch_to_metrics_map=batch_to_metrics_map,
            scalar_batch_metrics_map=scalar_batch_metrics_map,
        )

    metrics_proto = writer.write()
    with open(output_path, "wb") as f:
        f.write(metrics_proto.metrics_msg.SerializeToString())


def job_status_categories_metric(
    writer: ResimMetricsWriter,
    *,
    batches: list[Batch],
    batch_to_jobs_map: dict[str, list[Job]],
    job_to_metrics_map: dict[str, list[JobMetric]],
    batch_to_metrics_map: dict[str, list[BatchMetric]],
    scalar_batch_metrics_map: dict[str, dict[str, MetricProto]],
) -> None:
    # pylint: disable=unused-argument
    status_counts = _count_batch_statuses(
        list(batch_to_jobs_map.keys()), batch_to_jobs_map
    )

    fig = px.area(
        status_counts,
        x=range(len(batch_to_jobs_map)),
        y=["PASSED", "FAIL_WARN", "FAIL_BLOCK", "ERROR", "CANCELLED", "UNKNOWN"],
    )
    fig.update_layout(
        template="plotly_dark",
        plot_bgcolor="rgba(0, 0, 0, 0)",
        paper_bgcolor="rgba(0, 0, 0, 0)",
    )

    (
        writer.add_plotly_metric("Job Statuses Over Time")
        .with_description("Job Statuses Over Sequential Batches")
        .with_blocking(False)
        .with_should_display(True)
        .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
        .with_status(mu.MetricStatus.PASSED_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )

    fail_statuses = (MetricStatus.FAIL_BLOCK, MetricStatus.FAIL_WARN)

    def batch_is_failure(batch: Batch) -> bool:
        if batch.status == BatchStatus.ERROR:
            return True

        if batch.status != BatchStatus.SUCCEEDED:
            return False

        return (
            batch.batch_metrics_status in fail_statuses
            or batch.jobs_metrics_status in fail_statuses
        )

    def batch_is_success(batch: Batch) -> bool:
        if batch.status != BatchStatus.SUCCEEDED:
            return False

        return (
            batch.batch_metrics_status not in fail_statuses
            and batch.jobs_metrics_status not in fail_statuses
        )

    if len(batches) == 0:
        raise ValueError("Can't compute totals on zero batches!")

    batches_sorted = all(
        batches[i].creation_timestamp < batches[i + 1].creation_timestamp
        for i in range(len(batches) - 1)
    )

    if not batches_sorted:
        raise ValueError("Batches must be sorted!")

    totals = {
        "Number of Batches": len(batches),
        "Number of Jobs": len(job_to_metrics_map),
        "Number of Passing Batches": sum(batch_is_success(b) for b in batches),
        "Number of Failing Batches": sum(batch_is_failure(b) for b in batches),
        "Failed Jobs on Most Recent Batch": status_counts.tail(1)[
            ["FAIL_WARN", "FAIL_BLOCK", "ERROR"]
        ]
        .sum(axis=1)
        .iloc[0],
        "Passed Jobs on Most Recent Batch": status_counts.tail(1)["PASSED"].iloc[0],
    }

    totals_status: dict[str, mu.MetricStatus] = {}

    for k, v in totals.items():
        totals[k] = np.array([v], dtype=np.float64)
        totals_status[k] = np.array([mu.MetricStatus.PASSED_METRIC_STATUS])

    totals_data = rm.GroupedMetricsData(
        name="Totals Summary", category_to_series=totals
    )

    totals_status_data = rm.GroupedMetricsData(
        name="Totals Summary Statuses", category_to_series=totals_status
    )

    failure_def = mu.DoubleFailureDefinition(fails_below=None, fails_above=None)
    (
        writer.add_double_summary_metric("Totals")
        .with_description("High-level totals")
        .with_blocking(False)
        .with_should_display(True)
        .with_status(mu.MetricStatus.PASSED_METRIC_STATUS)
        .with_importance(mu.MetricImportance.ZERO_IMPORTANCE)
        .with_value_data(totals_data)
        .with_status_data(totals_status_data)
        .with_failure_definition(failure_def)
    )


def flaky_experiences_metric(
    writer: ResimMetricsWriter,
    *,
    batches: list[Batch],
    batch_to_jobs_map: dict[str, list[Job]],
    job_to_metrics_map: dict[str, list[JobMetric]],
    batch_to_metrics_map: dict[str, list[BatchMetric]],
    scalar_batch_metrics_map: dict[str, dict[str, MetricProto]],
) -> None:
    # pylint: disable=unused-argument

    fail_statuses = (MetricStatus.FAIL_BLOCK, MetricStatus.FAIL_WARN)

    def job_is_failure(job: Job) -> bool:
        if job.job_status == JobStatus.ERROR:
            return True

        if job.job_status != JobStatus.SUCCEEDED:
            return False

        return job.job_metrics_status in fail_statuses

    def metric_is_failure(metric: JobMetric) -> bool:
        return metric.status in fail_statuses

    def batch_metric_is_failure(metric: BatchMetric) -> bool:
        return metric.status in fail_statuses

    fail_counts: dict[str, int] = defaultdict(int)
    job_metric_fail_counts: dict[str, int] = defaultdict(int)
    batch_metric_fail_counts: dict[str, int] = defaultdict(int)
    for jobs in batch_to_jobs_map.values():
        for job in jobs:
            fail_counts[job.experience_name] += job_is_failure(job)
            for metric in job_to_metrics_map[job.job_id]:
                job_metric_fail_counts[metric.name] += metric_is_failure(metric)

    for batch in batches:
        for batch_metric in batch_to_metrics_map[batch.batch_id]:
            batch_metric_fail_counts[batch_metric.name] += batch_metric_is_failure(
                batch_metric
            )

    fail_counts_list = list(fail_counts.items())
    fail_counts_list.sort(key=lambda t: t[1], reverse=True)

    job_metric_fail_counts_list = list(job_metric_fail_counts.items())
    job_metric_fail_counts_list.sort(key=lambda t: t[1], reverse=True)

    batch_metric_fail_counts_list = list(batch_metric_fail_counts.items())
    batch_metric_fail_counts_list.sort(key=lambda t: t[1], reverse=True)

    fig = go.Figure(
        data=[
            go.Table(
                header={"values": ["Experience Name", "Fail Count"]},
                cells={
                    "values": [
                        [t[0] for t in fail_counts_list],
                        [t[1] for t in fail_counts_list],
                    ]
                },
            )
        ]
    )
    fig.update_layout(
        template="plotly_dark",
        plot_bgcolor="rgba(0, 0, 0, 0)",
        paper_bgcolor="rgba(0, 0, 0, 0)",
    )

    (
        writer.add_plotly_metric("Failures per Experience")
        .with_description("Experiences sorted by number of failures.")
        .with_blocking(False)
        .with_should_display(True)
        .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
        .with_status(mu.MetricStatus.PASSED_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )

    fig = go.Figure(
        data=[
            go.Table(
                header={"values": ["Metric Name", "Fail Count"]},
                cells={
                    "values": [
                        [t[0] for t in job_metric_fail_counts_list],
                        [t[1] for t in job_metric_fail_counts_list],
                    ]
                },
            )
        ]
    )
    fig.update_layout(
        template="plotly_dark",
        plot_bgcolor="rgba(0, 0, 0, 0)",
        paper_bgcolor="rgba(0, 0, 0, 0)",
    )

    (
        writer.add_plotly_metric("Failures per Job Metric")
        .with_description("Job Metrics sorted by number of failures.")
        .with_blocking(False)
        .with_should_display(True)
        .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
        .with_status(mu.MetricStatus.PASSED_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )

    fig = go.Figure(
        data=[
            go.Table(
                header={"values": ["Metric Name", "Fail Count"]},
                cells={
                    "values": [
                        [t[0] for t in batch_metric_fail_counts_list],
                        [t[1] for t in batch_metric_fail_counts_list],
                    ]
                },
            )
        ]
    )
    fig.update_layout(
        template="plotly_dark",
        plot_bgcolor="rgba(0, 0, 0, 0)",
        paper_bgcolor="rgba(0, 0, 0, 0)",
    )

    (
        writer.add_plotly_metric("Failures per Batch Metric")
        .with_description("Batch Metrics sorted by number of failures.")
        .with_blocking(False)
        .with_should_display(True)
        .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
        .with_status(mu.MetricStatus.PASSED_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )


def batch_metrics_scalars_over_time_metric(
    writer: ResimMetricsWriter,
    *,
    batches: list[Batch],
    batch_to_jobs_map: dict[str, list[Job]],
    job_to_metrics_map: dict[str, list[JobMetric]],
    batch_to_metrics_map: dict[str, list[BatchMetric]],
    scalar_batch_metrics_map: dict[str, dict[str, MetricProto]],
) -> None:
    # pylint: disable=unused-argument
    values: dict[str, list] = defaultdict(list)
    units: dict[str, str] = {}

    for i, batch in enumerate(batches):
        for metric in batch_to_metrics_map[batch.batch_id]:
            if metric.type == MetricType.SCALAR:
                values[metric.name].append([i, metric.value])
                metric = scalar_batch_metrics_map[batch.batch_id][metric.name]
                unit = metric.metric_values.scalar_metric_values.unit
                if metric.name in units and unit != units[metric.name]:
                    logger.warning("Inconsistent units for metric %s", metric.name)
                else:
                    units[metric.name] = unit

    array_values = {key: np.array(value) for (key, value) in values.items()}

    for key, value in array_values.items():
        index_data = rm.SeriesMetricsData(
            name=f"{key} Batch Index Data", series=value[:, 0]
        )

        value_data = rm.SeriesMetricsData(
            name=f"{key} Data", series=value[:, 1], unit=units[key]
        )

        status_data = rm.SeriesMetricsData(
            name=f"{key} statuses",
            series=np.array(
                [mu.MetricStatus.PASSED_METRIC_STATUS] * len(index_data.series)
            ),
        )
        (
            writer.add_line_plot_metric(f'"{key}" over time')
            .with_description(f'"{key}" collected from sequential test suite batches')
            .with_blocking(False)
            .with_should_display(True)
            .with_status(mu.MetricStatus.PASSED_METRIC_STATUS)
            .append_series_data(index_data, value_data, "Value over time")
            .append_statuses_data(status_data)
            .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
            .with_x_axis_name("Batch Index Over Time")
            .with_y_axis_name("Metric Value")
        )


def _count_batch_statuses(
    batch_ids: list[str], batch_to_jobs_map: dict[str, list[Job]]
) -> pd.DataFrame:
    status_counts = []
    for batch_id in batch_ids:
        status_counts.append(_count_job_statuses(batch_to_jobs_map[batch_id]))

    status_counts = pd.DataFrame(
        data=np.array(status_counts),
        columns=["PASSED", "FAIL_WARN", "FAIL_BLOCK", "ERROR", "CANCELLED", "UNKNOWN"],
        index=range(len(batch_ids)),
    )

    return status_counts


def _count_job_statuses(jobs: list[Job]) -> list[int]:
    counts: dict[JobStatus, MetricStatus] = defaultdict(int)
    for job in jobs:
        counts[(job.job_status, job.job_metrics_status)] += 1

    passed_count = counts[(JobStatus.SUCCEEDED, MetricStatus.PASSED)]
    fail_warn_count = counts[(JobStatus.SUCCEEDED, MetricStatus.FAIL_WARN)]
    fail_block_count = counts[(JobStatus.SUCCEEDED, MetricStatus.FAIL_BLOCK)]
    error_count = sum(counts[JobStatus.ERROR, ms] for ms in MetricStatus)
    cancelled_count = sum(counts[JobStatus.CANCELLED, ms] for ms in MetricStatus)

    categorized_counts = [
        passed_count,
        fail_warn_count,
        fail_block_count,
        error_count,
        cancelled_count,
    ]
    # Add unknown column
    categorized_counts.append(len(jobs) - sum(categorized_counts))

    return categorized_counts


def _write_proto(writer: ResimMetricsWriter) -> None:
    metrics_proto = writer.write()
    validate_job_metrics(metrics_proto.metrics_msg)
    # Known location where the runner looks for metrics
    with open("/tmp/resim/outputs/metrics.binproto", "wb") as f:
        f.write(metrics_proto.metrics_msg.SerializeToString())


if __name__ == "__main__":
    asyncio.run(main())
