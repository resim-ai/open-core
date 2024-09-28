# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


import argparse
import asyncio
import json
import logging
import pathlib
import sys
import uuid
from collections import defaultdict
from typing import Awaitable, Hashable

import httpx
import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from resim_python_client.api.batches import list_batch_metrics, list_metrics_for_job
from resim_python_client.api.builds import get_build
from resim_python_client.api.experiences import list_experience_tags_for_experience
from resim_python_client.client import AuthenticatedClient
from resim_python_client.models.batch import Batch
from resim_python_client.models.batch_metric import BatchMetric
from resim_python_client.models.batch_status import BatchStatus
from resim_python_client.models.conflated_job_status import ConflatedJobStatus
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
from resim.metrics.resim_style import resim_plotly_style, resim_status_color_map

logger = logging.getLogger("resim")

DEFAULT_CONFIG_PATH = pathlib.Path("/tmp/resim/inputs/report_config.json")
DEFAULT_OUTPUT_PATH = pathlib.Path("/tmp/resim/outputs/metrics.binproto")


async def dict_gather(coroutine_dict: dict) -> dict:
    """A simple helper function that enables asyncio.gather to be called on dictionaries."""

    async def tag(key: Hashable, coroutine: Awaitable) -> tuple:
        return key, await coroutine

    return dict(
        await asyncio.gather(
            *(tag(key, coroutine) for (key, coroutine) in coroutine_dict.items())
        )
    )


################################################################################
# WIP
################################################################################


def make_metrics(
    *,
    writer: ResimMetricsWriter,
    tests_frame: pd.DataFrame,
    builds_frame: pd.DataFrame,
    tags_frame: pd.DataFrame,
    job_metrics_frame: pd.DataFrame,
    batch_metrics_frame: pd.DataFrame,
):
    fig = go.Figure()

    status_counts_frame = (
        tests_frame.groupby("build_id")["status"]
        .value_counts()
        .unstack(fill_value=0)
        .join(builds_frame)
    )

    for status in (
        ConflatedJobStatus.PASSED,
        ConflatedJobStatus.WARNING,
        ConflatedJobStatus.BLOCKER,
        ConflatedJobStatus.ERROR,
        ConflatedJobStatus.CANCELLED,
    ):
        if status not in status_counts_frame.columns:
            continue
        fig.add_trace(
            go.Bar(
                x=status_counts_frame["build_creation_timestamp"],
                y=status_counts_frame[status],
                name=status,
                marker_color=resim_status_color_map[status],
            )
        )

    resim_plotly_style(
        fig,
        barmode="stack",
        yaxis_title="Number of Tests",
        xaxis_title="Time",
    )

    (
        writer.add_plotly_metric("test_status_over_time")
        .with_description("Test Statuses Over Sequential Batches")
        .with_blocking(False)
        .with_should_display(True)
        .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
        .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )

    print(job_metrics_frame["name"])

    for i in range(20):
        (
            writer.add_scalar_metric(f"metric_{i}")
            .with_description("")
            .with_blocking(False)
            .with_should_display(True)
            .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
            .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
            .with_value(0.0)
        )


################################################################################
# FETCHERS
################################################################################


async def _fetch_job_metrics_for_job(
    client: AuthenticatedClient, *, project_id: str, batch_id: str, job_id: str
) -> list[JobMetric]:
    """Fetch job metrics for a given job.

    Args:
        client: A client for performing these queries.
        project_id: The id for the project for this job.
        batch_id: The id for the batch of this job.
        job_id: This job's id

    Returns:
        A list of this job's Metrics.
    """
    job_metrics = await async_fetch_all_pages(
        list_metrics_for_job.asyncio,
        project_id=project_id,
        batch_id=batch_id,
        job_id=job_id,
        client=client,
    )
    return [j for page in job_metrics for j in page.metrics]


async def fetch_job_metrics(
    client: AuthenticatedClient,
    *,
    project_id: uuid.UUID,
    batch_to_jobs_map: dict[str, list[Job]],
) -> dict[str, list[JobMetric]]:
    """Fetch job metrics for all of our batches.

    Args:
        client: A client for performing these queries.
        project_id: The id for the project for this job.
        batch_to_job_map: A map containing the Jobs for each batch.

    Returns:
        A dictionary of job_ids to a list of metrics for all batch's jobs.
    """
    return await dict_gather(
        {
            job.job_id: _fetch_job_metrics_for_job(
                client, project_id=str(project_id), batch_id=batch_id, job_id=job.job_id
            )
            for batch_id, jobs in batch_to_jobs_map.items()
            for job in jobs
        }
    )


async def _fetch_batch_metrics_for_batch(
    client: AuthenticatedClient, *, project_id: str, batch_id: str
) -> list[BatchMetric]:
    """Fetch batch metrics for a single batch.

    Args:
        client: A client for performing these queries.
        project_id: The id for the project for this batch.
        batch_id: The id for the batch whose metrics we want to fetch.

    Returns:
        A list of batches metrics for this batch.
    """
    batch_metrics = await async_fetch_all_pages(
        list_batch_metrics.asyncio,
        project_id=project_id,
        batch_id=batch_id,
        client=client,
    )
    return [b for page in batch_metrics for b in page.batch_metrics]


async def fetch_batch_metrics(
    client: AuthenticatedClient,
    *,
    project_id: uuid.UUID,
    batch_ids: list[str],
) -> dict[str, list[BatchMetric]]:
    """Fetch batch metrics for all of our batches.

    Args:
        client: A client for performing these queries.
        project_id: The id for the project for this batch.
        batch_ids: The ids for all the batches we need metrics for.

    Returns:
        A dictionary of batch_ids to batch metrics for all of our batches.
    """
    return await dict_gather(
        {
            batch_id: _fetch_batch_metrics_for_batch(
                client, project_id=str(project_id), batch_id=batch_id
            )
            for batch_id in batch_ids
        }
    )


async def fetch_scalar_batch_metrics(
    *, batch_to_metrics_map: dict[str, list[BatchMetric]]
) -> dict[str, dict[str, MetricProto]]:
    """Fetch all scalar metric protos for all of our batches.

    Scalar metrics for each batch are very useful to display in reports and we
    need to fetch and unpack the protos to get the units from them. This
    function does this.

    Args:
        batch_to_metrics_map: A map containing all of the batch metrics for each
                              batch. Keyed by batch id.
    Returns:
        A dictionary of batch_id to dict of batch metric name to metric proto
        object.
    """
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


def _count_job_statuses(jobs: list[Job]) -> list[int]:
    """Count the different output statuses in a given job list.

    Count job statuses into the following categories in order:

    PASSED
    FAIL_WARN
    FAIL_BLOCK
    ERROR
    CANCELLED
    UNKNOWN

    Args:
        jobs: The jobs whose statuses we care about.

    Returns:
        A list of counts in the aforementioned order.
    """
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


def _count_batch_statuses(
    batch_ids: list[str], batch_to_jobs_map: dict[str, list[Job]]
) -> pd.DataFrame:
    """Count the different output statuses of jobs for each batch.

    Returns a pandas dataframe where each row is a batch in order of timestamp
    and each column is a status like so:

           PASSED  FAIL_WARN  FAIL_BLOCK  ERROR  CANCELLED  UNKNOWN
        0      22         10          18      0          0        0
        1      23         10          17      0          0        0
        2      23         10          17      0          0        0
        3      22         10          18      0          0        0
        4      21         10          19      0          0        0

    Args:
        batch_ids: The batches whose statuses we care about.
        batch_to_jobs_map: All of the jobs per batch.

    Returns:
        A dataframe containing status counts where each row is a batch in order
        and each column is a status.

    """
    status_counts = []
    for batch_id in batch_ids:
        status_counts.append(_count_job_statuses(batch_to_jobs_map[batch_id]))

    status_counts = pd.DataFrame(
        data=np.array(status_counts),
        columns=["PASSED", "FAIL_WARN", "FAIL_BLOCK", "ERROR", "CANCELLED", "UNKNOWN"],
        index=range(len(batch_ids)),
    )
    return status_counts


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
    """Create a variety of job status metrics.

    Includes a filled line plot of statuses alongside aggregate metrics on:
     - Number of batches
     - Number of jobs
     - Number of passing batches
     - Number of failing batches
     - Failed jobs on most recent batch
     - etc.

    Args:
        writer: A metrics writer to write these metrics to.
        batches: All batches for this report.
        batch_to_jobs_map: All jobs for each batch.
        job_to_metrics_map: All job metrics for each job.
        batch_to_metrics_map: All batch metrics for each batch.
        scalar_batch_metrics_map: All scalar batch metrics for each batch as protos.
    """
    status_counts = _count_batch_statuses(
        [b.batch_id for b in batches], batch_to_jobs_map
    )

    status_counts["Batch Number"] = list(range(len(batch_to_jobs_map)))

    fig = px.area(
        status_counts,
        x="Batch Number",
        y=["PASSED", "FAIL_WARN", "FAIL_BLOCK", "ERROR", "CANCELLED", "UNKNOWN"],
    )
    fig.update_layout(
        template="plotly_dark",
        plot_bgcolor="rgba(0, 0, 0, 0)",
        yaxis_title="Number of Tests",
        paper_bgcolor="rgba(0, 0, 0, 0)",
    )

    (
        writer.add_plotly_metric("Job Statuses Over Time")
        .with_description("Job Statuses Over Sequential Batches")
        .with_blocking(False)
        .with_should_display(True)
        .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
        .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )

    fail_statuses = (MetricStatus.FAIL_BLOCK, MetricStatus.FAIL_WARN)

    def batch_is_fail_block(batch: Batch) -> bool:
        if batch.status != BatchStatus.SUCCEEDED:
            return False

        return MetricStatus.FAIL_BLOCK in (
            batch.batch_metrics_status,
            batch.jobs_metrics_status,
        )

    def batch_is_fail_warn(batch: Batch) -> bool:
        if batch.status != BatchStatus.SUCCEEDED:
            return False

        result: bool = not batch_is_fail_block(batch) and (
            batch.batch_metrics_status in fail_statuses
            or batch.jobs_metrics_status in fail_statuses
        )
        return result

    def batch_is_error(batch: Batch) -> bool:
        result: bool = batch.status == BatchStatus.ERROR
        return result

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
        "Number of Fail Warn Batches": sum(batch_is_fail_warn(b) for b in batches),
        "Number of Fail Block Batches": sum(batch_is_fail_block(b) for b in batches),
        "Number of Error Batches": sum(batch_is_error(b) for b in batches),
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
        totals_status[k] = np.array([mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS])

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
        .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
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
    """Creates metrics listing flaky experiences and metrics

    Creates a plotly table describing the experiences that failed most in this
    report, job metrics that failed most, and batch metrics that failed most.

    Args:
        writer: A metrics writer to write these metrics to.
        batches: All batches for this report.
        batch_to_jobs_map: All jobs for each batch.
        job_to_metrics_map: All job metrics for each job.
        batch_to_metrics_map: All batch metrics for each batch.
        scalar_batch_metrics_map: All scalar batch metrics for each batch as protos.
    """

    def job_is_error(job: Job) -> bool:
        result: bool = job.job_status == JobStatus.ERROR
        return result

    def job_is_fail_block(job: Job) -> bool:
        if job.job_status != JobStatus.SUCCEEDED:
            return False

        result: bool = job.job_metrics_status == MetricStatus.FAIL_BLOCK
        return result

    def job_is_fail_warn(job: Job) -> bool:
        if job.job_status != JobStatus.SUCCEEDED:
            return False

        result: bool = job.job_metrics_status == MetricStatus.FAIL_WARN
        return result

    def metric_is_fail_block(metric: JobMetric) -> bool:
        result: bool = metric.status == MetricStatus.FAIL_BLOCK
        return result

    def metric_is_fail_warn(metric: JobMetric) -> bool:
        result: bool = metric.status == MetricStatus.FAIL_WARN
        return result

    def batch_metric_is_fail_block(metric: BatchMetric) -> bool:
        result: bool = metric.status == MetricStatus.FAIL_BLOCK
        return result

    def batch_metric_is_fail_warn(metric: BatchMetric) -> bool:
        result: bool = metric.status == MetricStatus.FAIL_WARN
        return result

    fail_error_counts: dict[str, int] = defaultdict(int)
    fail_block_counts: dict[str, int] = defaultdict(int)
    fail_warn_counts: dict[str, int] = defaultdict(int)

    job_metric_fail_block_counts: dict[str, int] = defaultdict(int)
    job_metric_fail_warn_counts: dict[str, int] = defaultdict(int)

    batch_metric_fail_block_counts: dict[str, int] = defaultdict(int)
    batch_metric_fail_warn_counts: dict[str, int] = defaultdict(int)

    for jobs in batch_to_jobs_map.values():
        for job in jobs:
            fail_error_counts[job.experience_name] += job_is_error(job)
            fail_block_counts[job.experience_name] += job_is_fail_block(job)
            fail_warn_counts[job.experience_name] += job_is_fail_warn(job)
            for metric in job_to_metrics_map[job.job_id]:
                job_metric_fail_block_counts[metric.name] += metric_is_fail_block(
                    metric
                )
                job_metric_fail_warn_counts[metric.name] += metric_is_fail_warn(metric)

    for batch in batches:
        for batch_metric in batch_to_metrics_map[batch.batch_id]:
            batch_metric_fail_block_counts[
                batch_metric.name
            ] += batch_metric_is_fail_block(batch_metric)
            batch_metric_fail_warn_counts[
                batch_metric.name
            ] += batch_metric_is_fail_warn(batch_metric)

    fail_counts = {
        job_name: fail_error_counts[job_name]
        + fail_warn_counts[job_name]
        + fail_block_counts[job_name]
        for job_name in fail_error_counts
    }
    fail_counts_list = [
        (
            exp_name,
            count,
            fail_error_counts[exp_name],
            fail_block_counts[exp_name],
            fail_warn_counts[exp_name],
        )
        for exp_name, count in fail_counts.items()
    ]
    fail_counts_list.sort(key=lambda t: t[1:], reverse=True)

    job_metric_fail_counts = {
        metric_name: job_metric_fail_block_counts[metric_name]
        + job_metric_fail_warn_counts[metric_name]
        for metric_name in job_metric_fail_block_counts
    }
    job_metric_fail_counts_list = [
        (
            name,
            count,
            job_metric_fail_block_counts[name],
            job_metric_fail_warn_counts[name],
        )
        for name, count in job_metric_fail_counts.items()
    ]
    job_metric_fail_counts_list.sort(key=lambda t: t[1:], reverse=True)

    batch_metric_fail_counts = {
        metric_name: batch_metric_fail_block_counts[metric_name]
        + batch_metric_fail_warn_counts[metric_name]
        for metric_name in batch_metric_fail_block_counts
    }
    batch_metric_fail_counts_list = [
        (
            name,
            count,
            batch_metric_fail_block_counts[name],
            batch_metric_fail_warn_counts[name],
        )
        for name, count in batch_metric_fail_counts.items()
    ]
    batch_metric_fail_counts_list.sort(key=lambda t: t[1:], reverse=True)

    fig = go.Figure(
        data=[
            go.Table(
                header={
                    "values": [
                        "Experience Name",
                        "Fail Count",
                        "Error Count",
                        "Fail Block Count",
                        "Fail Warn Count",
                    ]
                },
                cells={
                    "values": [
                        [t[0] for t in fail_counts_list],
                        [t[1] for t in fail_counts_list],
                        [t[2] for t in fail_counts_list],
                        [t[3] for t in fail_counts_list],
                        [t[4] for t in fail_counts_list],
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
        .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )

    fig = go.Figure(
        data=[
            go.Table(
                header={
                    "values": [
                        "Metric Name",
                        "Fail Count",
                        "Fail Block Count",
                        "Fail Warn Count",
                    ]
                },
                cells={
                    "values": [
                        [t[0] for t in job_metric_fail_counts_list],
                        [t[1] for t in job_metric_fail_counts_list],
                        [t[2] for t in job_metric_fail_counts_list],
                        [t[3] for t in job_metric_fail_counts_list],
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
        .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )

    fig = go.Figure(
        data=[
            go.Table(
                header={
                    "values": [
                        "Metric Name",
                        "Fail Count",
                        "Fail Block Count",
                        "Fail Warn Count",
                    ]
                },
                cells={
                    "values": [
                        [t[0] for t in batch_metric_fail_counts_list],
                        [t[1] for t in batch_metric_fail_counts_list],
                        [t[2] for t in batch_metric_fail_counts_list],
                        [t[3] for t in batch_metric_fail_counts_list],
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
        .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
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
    """Creates metrics plotting batch metric scalars over time.

    Creates a line plot showing the value over time for each scalar batch metric
    defined by this report's test suite's metrics build.

    Args:
        writer: A metrics writer to write these metrics to.
        batches: All batches for this report.
        batch_to_jobs_map: All jobs for each batch.
        job_to_metrics_map: All job metrics for each job.
        batch_to_metrics_map: All batch metrics for each batch.
        scalar_batch_metrics_map: All scalar batch metrics for each batch as protos.
    """
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
                [mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS] * len(index_data.series)
            ),
        )
        (
            writer.add_line_plot_metric(f'"{key}" over time')
            .with_description(f'"{key}" collected from sequential test suite batches')
            .with_blocking(False)
            .with_should_display(True)
            .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
            .append_series_data(index_data, value_data, "Value over time")
            .append_statuses_data(status_data)
            .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
            .with_x_axis_name("Batch Index Over Time")
            .with_y_axis_name("Metric Value")
        )


async def compute_metrics(
    *,
    token: str,
    api_url: str,
    project_id: uuid.UUID,
    report_id: uuid.UUID,
    output_path: str,
) -> None:
    """Compute all default report metrics for this report id.

    Args:
        token: An auth token used to compute metrics.
        api_url: The api url to fetch data from.
        project_id: The project for this report.
        report_id: The id of this report.
        output_path: The place to save the metrics binary proto blob.
    """
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

    tests_frame = pd.DataFrame(
        [
            (
                t.job_id,
                t.batch_id,
                t.build_id,
                t.experience_id,
                t.experience_name,
                t.conflated_status,
            )
            for tests in batch_to_jobs_map.values()
            for t in tests
        ],
        columns=[
            "job_id",
            "batch_id",
            "build_id",
            "experience_id",
            "experience_name",
            "status",
        ],
    )
    tests_frame.set_index("job_id", inplace=True)

    build_ids = set(tests_frame["build_id"])

    async def get_build_row(build_id: str):
        build = await get_build.asyncio(
            client=client, project_id=project_id, build_id=build_id
        )
        return build_id, build.creation_timestamp

    builds_frame = pd.DataFrame(
        await asyncio.gather(*[get_build_row(b) for b in build_ids]),
        columns=["build_id", "build_creation_timestamp"],
    )
    builds_frame.set_index("build_id", inplace=True)

    experience_ids = set(tests_frame["experience_id"])

    async def get_tag_row(experience_id: str):
        pages = await async_fetch_all_pages(
            list_experience_tags_for_experience.asyncio,
            client=client,
            project_id=project_id,
            experience_id=experience_id,
        )
        return experience_id, {e.name for page in pages for e in page.experience_tags}

    tags_frame = pd.DataFrame(
        await asyncio.gather(*[get_tag_row(e) for e in experience_ids]),
        columns=["experience_id", "tags"],
    )
    tags_frame.set_index("experience_id", inplace=True)

    job_metrics_frame = pd.DataFrame(
        [
            (m.job_id, m.name, m.value, m.unit, m.type, m.status)
            for metrics in job_to_metrics_map.values()
            for m in metrics
        ],
        columns=["job_id", "name", "value", "unit", "type", "status"],
    )
    job_metrics_frame.set_index(["name", "job_id"], inplace=True)
    job_metrics_frame.sort_index(inplace=True)

    batch_metrics_frame = pd.DataFrame(
        [
            (m.batch_id, m.name, m.value, m.unit, m.type, m.status)
            for metrics in batch_to_metrics_map.values()
            for m in metrics
        ],
        columns=["batch_id", "name", "value", "unit", "type", "status"],
    )
    batch_metrics_frame.set_index(["name", "batch_id"], inplace=True)
    batch_metrics_frame.sort_index(inplace=True)

    writer = ResimMetricsWriter(uuid.uuid4())  # Make metrics writer!

    make_metrics(
        writer=writer,
        tests_frame=tests_frame,
        builds_frame=builds_frame,
        tags_frame=tags_frame,
        job_metrics_frame=job_metrics_frame,
        batch_metrics_frame=batch_metrics_frame,
    )

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
    validate_job_metrics(metrics_proto.metrics_msg)
    with open(output_path, "wb") as f:
        f.write(metrics_proto.metrics_msg.SerializeToString())


async def main() -> None:
    """Compute all default report metrics based on the report config path passed in."""
    logging.basicConfig()
    logger.setLevel(logging.INFO)

    parser = argparse.ArgumentParser(
        prog="default_report_metrics",
        description="Compute a basic set of report metrics.",
    )
    parser.add_argument("--report-config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--output-path", default=str(DEFAULT_OUTPUT_PATH))

    args = parser.parse_args()

    report_metrics_config_path = pathlib.Path(args.report_config)
    is_report_metrics_mode = report_metrics_config_path.is_file()
    if is_report_metrics_mode:
        with open(args.report_config, "r", encoding="utf-8") as f:
            report_config = json.load(f)

        await compute_metrics(
            token=report_config["authToken"],
            api_url=report_config["apiURL"],
            project_id=uuid.UUID(report_config["projectID"]),
            report_id=uuid.UUID(report_config["reportID"]),
            output_path=args.output_path,
        )

    else:
        # Otherwise, we can't do anything: exit 1
        print("Report config does not exist. Are you sure you're in report mode?")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
