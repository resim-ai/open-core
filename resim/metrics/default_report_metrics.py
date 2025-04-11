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
from datetime import datetime
from typing import Awaitable, Hashable

import numpy as np
import pandas as pd
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
from resim_python_client.models.metric_status import MetricStatus
from resim_python_client.models.metric_type import MetricType

from resim_python_client.models import (
    ListJobMetricsOutput,
    ListBatchMetricsOutput,
    ListExperiencesOutput,
)

import resim.metrics.python.metrics_utils as mu
from resim.metrics.fetch_all_pages import async_fetch_all_pages
from resim.metrics.fetch_report_metrics import (
    fetch_batches_for_report,
    fetch_jobs_for_batches,
)
from resim.metrics.proto.validate_metrics_proto import validate_job_metrics
from resim.metrics.python.metrics import SeriesMetricsData
from resim.metrics.python.metrics_writer import ResimMetricsWriter
from resim.metrics.resim_style import resim_plotly_style, resim_status_color_map

logger = logging.getLogger("resim")

DEFAULT_CONFIG_PATH = pathlib.Path("/tmp/resim/inputs/report_config.json")
DEFAULT_OUTPUT_PATH = pathlib.Path("/tmp/resim/outputs/metrics.binproto")


################################################################################
# HELPERS
################################################################################


async def dict_gather(coroutine_dict: dict) -> dict:
    """A simple helper function that enables asyncio.gather to be called on dictionaries."""
    semaphore = asyncio.Semaphore(24)

    async def tag(key: Hashable, coroutine: Awaitable) -> tuple:
        async with semaphore:
            return key, await coroutine

    return dict(
        await asyncio.gather(
            *(tag(key, coroutine) for (key, coroutine) in coroutine_dict.items())
        )
    )


# fmt: off
CONFLATED_STATUS_MAPPING = {
    BatchStatus.BATCH_METRICS_QUEUED:  ConflatedJobStatus.RUNNING,
    BatchStatus.BATCH_METRICS_RUNNING: ConflatedJobStatus.RUNNING,
    BatchStatus.EXPERIENCES_RUNNING:   ConflatedJobStatus.RUNNING,
    BatchStatus.CANCELLED:             ConflatedJobStatus.CANCELLED,
    BatchStatus.ERROR:                 ConflatedJobStatus.ERROR,
    BatchStatus.SUBMITTED:             ConflatedJobStatus.QUEUED,
}
# fmt: on


def batch_conflated_status(batch: pd.DataFrame) -> ConflatedJobStatus:
    """Compute a conflated status based on a row from the batches frame."""
    if batch.batch_status in CONFLATED_STATUS_MAPPING:
        return CONFLATED_STATUS_MAPPING[batch.batch_status]
    if (
        MetricStatus.FAIL_BLOCK
        in batch[["batch_metrics_status", "jobs_metrics_status"]].to_list()
    ):
        return ConflatedJobStatus.BLOCKER
    if (
        MetricStatus.FAIL_WARN
        in batch[["batch_metrics_status", "jobs_metrics_status"]].to_list()
    ):
        return ConflatedJobStatus.WARNING
    return ConflatedJobStatus.PASSED


################################################################################
# FETCHERS
################################################################################


async def _fetch_job_metrics_for_job(
    client: AuthenticatedClient, *, project_id: str, batch_id: str, job_id: str
) -> list[JobMetric]:
    """Fetch job metrics for a given job.

    Args:
        client:     A client for performing these queries.
        project_id: The id for the project for this job.
        batch_id:   The id for the batch of this job.
        job_id:     This job's id

    Returns:
        A list of this job's Metrics.
    """
    job_metrics: list[ListJobMetricsOutput] = await async_fetch_all_pages(
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
        client:           A client for performing these queries.
        project_id:       The id for the project for this job.
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
        client:     A client for performing these queries.
        project_id: The id for the project for this batch.
        batch_id:   The id for the batch whose metrics we want to fetch.

    Returns:
        A list of batches metrics for this batch.
    """
    batch_metrics: list[ListBatchMetricsOutput] = await async_fetch_all_pages(
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
        client:     A client for performing these queries.
        project_id: The id for the project for this batch.
        batch_ids:  The ids for all the batches we need metrics for.

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


def make_batches_frame(batches: list[Batch]) -> pd.DataFrame:
    batches_frame = pd.DataFrame(
        [
            (
                b.batch_id,
                b.build_id,
                b.batch_metrics_status,
                b.jobs_metrics_status,
                b.status,
            )
            for b in batches
        ],
        columns=[
            "batch_id",
            "build_id",
            "batch_metrics_status",
            "jobs_metrics_status",
            "batch_status",
        ],
    )
    batches_frame.set_index("batch_id", inplace=True)
    batches_frame["status"] = batches_frame.apply(batch_conflated_status, axis=1)
    return batches_frame


def make_tests_frame(batch_to_jobs_map: dict[str, list[Job]]) -> pd.DataFrame:
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
    return tests_frame


async def make_builds_frame(
    batches: list[Batch], *, client: AuthenticatedClient, project_id: uuid.UUID
) -> pd.DataFrame:
    build_ids = {b.build_id for b in batches}

    async def get_build_row(build_id: str) -> tuple[str, datetime]:
        build = await get_build.asyncio(
            client=client, project_id=str(project_id), build_id=build_id
        )
        return build_id, build.creation_timestamp

    builds_frame = pd.DataFrame(
        await asyncio.gather(*[get_build_row(b) for b in build_ids]),
        columns=["build_id", "build_creation_timestamp"],
    )
    builds_frame.set_index("build_id", inplace=True)
    return builds_frame


async def make_tags_frame(
    batch_to_jobs_map: dict[str, list[Job]],
    *,
    client: AuthenticatedClient,
    project_id: uuid.UUID,
) -> pd.DataFrame:
    experience_ids = {
        j.experience_id for jobs in batch_to_jobs_map.values() for j in jobs
    }

    async def get_tag_row(experience_id: str) -> tuple[str, set[str]]:
        pages: list[ListExperiencesOutput] = await async_fetch_all_pages(
            list_experience_tags_for_experience.asyncio,
            client=client,
            project_id=str(project_id),
            experience_id=experience_id,
        )
        return experience_id, {e.name for page in pages for e in page.experience_tags}

    tags_frame = pd.DataFrame(
        await asyncio.gather(*[get_tag_row(e) for e in experience_ids]),
        columns=["experience_id", "tags"],
    )
    tags_frame.set_index("experience_id", inplace=True)
    return tags_frame


def make_job_metrics_frame(
    job_to_metrics_map: dict[str, list[JobMetric]],
) -> pd.DataFrame:
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
    return job_metrics_frame


def make_batch_metrics_frame(
    batch_to_metrics_map: dict[str, list[BatchMetric]],
) -> pd.DataFrame:
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
    return batch_metrics_frame


################################################################################


async def fetch_and_compute_metrics(
    *,
    token: str,
    api_url: str,
    project_id: uuid.UUID,
    report_id: uuid.UUID,
    output_path: str,
) -> None:
    """Compute all default report metrics for this report id.

    Args:
        token:       An auth token used to compute metrics.
        api_url:     The api url to fetch data from.
        project_id:  The project for this report.
        report_id:   The id of this report.
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

    logger.info(
        "Fetched metrics for %d jobs and %d batches",
        len(job_to_metrics_map),
        len(batch_to_metrics_map),
    )

    if len(batches) == 0:
        return

    ##############################
    # Assemble frames
    ##############################

    batches_frame = make_batches_frame(batches)
    tests_frame = make_tests_frame(batch_to_jobs_map)
    builds_frame = await make_builds_frame(
        batches, client=client, project_id=project_id
    )
    tags_frame = await make_tags_frame(
        batch_to_jobs_map, client=client, project_id=project_id
    )
    job_metrics_frame = make_job_metrics_frame(job_to_metrics_map)
    batch_metrics_frame = make_batch_metrics_frame(batch_to_metrics_map)

    writer = ResimMetricsWriter(uuid.uuid4())  # Make metrics writer!

    compute_metrics(
        writer=writer,
        project_id=project_id,
        batches_frame=batches_frame,
        tests_frame=tests_frame,
        builds_frame=builds_frame,
        tags_frame=tags_frame,
        job_metrics_frame=job_metrics_frame,
        batch_metrics_frame=batch_metrics_frame,
    )

    metrics_proto = writer.write()
    validate_job_metrics(metrics_proto.metrics_msg)
    with open(output_path, "wb") as f:
        f.write(metrics_proto.metrics_msg.SerializeToString())


def to_resim_timestamp(ts: pd.Timestamp) -> mu.Timestamp:
    NANOS_PER_SEC = 1e9
    return mu.Timestamp(
        secs=int(ts.timestamp()),
        nanos=int((ts.timestamp() - int(ts.timestamp())) * NANOS_PER_SEC),
    )


def test_status_over_time_metric(
    *,
    writer: ResimMetricsWriter,
    project_id: uuid.UUID,
    tests_frame: pd.DataFrame,
    builds_frame: pd.DataFrame,
) -> None:
    status_counts_frame = (
        tests_frame.groupby(["batch_id", "build_id"])["status"]
        .value_counts()
        .unstack(fill_value=0)
        .join(builds_frame)
        .sort_values(by=["build_creation_timestamp"])
        .reset_index()
    )

    statuses = (
        ConflatedJobStatus.PASSED,
        ConflatedJobStatus.WARNING,
        ConflatedJobStatus.BLOCKER,
        ConflatedJobStatus.ERROR,
        ConflatedJobStatus.CANCELLED,
    )

    status_over_time_metric = (
        writer.add_batchwise_bar_chart_metric(name="test_status_over_time")
        .with_description("Test Statuses Over Sequential Batches")
        .with_blocking(False)
        .with_should_display(True)
        .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
        .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
        .with_project_id(project_id)
        .with_stack_bars(True)
        .with_x_axis_name("Time")
        .with_y_axis_name("Number of Experiences")
    )

    index_data = SeriesMetricsData(
        name="batch_ids",
        series=np.array(
            [uuid.UUID(id) for id in status_counts_frame.batch_id.to_list()]
        ),
    )

    colors = []

    for status in statuses:
        if status not in status_counts_frame.columns:
            continue

        status_over_time_metric.append_category_data(
            category=status,
            times_data=SeriesMetricsData(
                name=f"Batch Times: {status}",
                series=np.array(
                    list(
                        map(
                            to_resim_timestamp,
                            status_counts_frame.build_creation_timestamp.to_list(),
                        )
                    )
                ),
                index_data=index_data,
            ),
            values_data=SeriesMetricsData(
                name=f"Status Counts: {status}",
                series=np.array(
                    status_counts_frame[status].to_numpy().astype(np.float64)
                ),
                index_data=index_data,
            ),
            statuses_data=SeriesMetricsData(
                name=f"Statuses: {status}",
                series=np.array(
                    [mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS]
                    * len(status_counts_frame)
                ),
                index_data=index_data,
            ),
        )

        colors.append(resim_status_color_map[status])

    status_over_time_metric.with_colors(colors)


def high_level_totals_metrics(
    *,
    writer: ResimMetricsWriter,
    batches_frame: pd.DataFrame,
    tests_frame: pd.DataFrame,
) -> None:
    batch_status_counts_frame = batches_frame["status"].value_counts()
    for status in (
        ConflatedJobStatus.PASSED,
        ConflatedJobStatus.WARNING,
        ConflatedJobStatus.BLOCKER,
        ConflatedJobStatus.ERROR,
        ConflatedJobStatus.CANCELLED,
    ):
        if status in batch_status_counts_frame:
            continue
        batch_status_counts_frame[status] = 0

    fig = go.Figure(
        data=[
            go.Table(
                header={"values": ["Name", "Value"]},
                cells={
                    "values": [
                        [
                            "Number of batches",
                            "Number of tests",
                            "Number of error batches",
                            "Number of fail block batches",
                            "Number of fail warn batches",
                            "Number of passed batches",
                        ],
                        [
                            len(batches_frame),
                            len(tests_frame),
                            batch_status_counts_frame[ConflatedJobStatus.ERROR],
                            batch_status_counts_frame[ConflatedJobStatus.BLOCKER],
                            batch_status_counts_frame[ConflatedJobStatus.WARNING],
                            batch_status_counts_frame[ConflatedJobStatus.PASSED],
                        ],
                    ]
                },
            )
        ]
    )

    resim_plotly_style(
        fig,
    )

    (
        writer.add_plotly_metric("high_level_totals")
        .with_description("High-level totals")
        .with_blocking(False)
        .with_should_display(True)
        .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
        .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )


def experience_status_counts_metric(
    *,
    writer: ResimMetricsWriter,
    tests_frame: pd.DataFrame,
) -> None:
    experience_names_frame = (
        tests_frame[["experience_id", "experience_name"]]
        .drop_duplicates()
        .set_index("experience_id")
    )

    experience_status_counts_frame = (
        tests_frame.groupby("experience_id")["status"]
        .value_counts()
        .unstack(fill_value=0)
        .join(experience_names_frame)
        .set_index("experience_name")
    )

    for status in (
        ConflatedJobStatus.PASSED,
        ConflatedJobStatus.WARNING,
        ConflatedJobStatus.BLOCKER,
        ConflatedJobStatus.ERROR,
        ConflatedJobStatus.CANCELLED,
    ):
        if status in experience_status_counts_frame.columns:
            continue
        experience_status_counts_frame[status] = 0

    experience_status_counts_frame.sort_values(
        by=[
            ConflatedJobStatus.ERROR,
            ConflatedJobStatus.BLOCKER,
            ConflatedJobStatus.WARNING,
            ConflatedJobStatus.CANCELLED,
            ConflatedJobStatus.PASSED,
        ],
        ascending=False,
        inplace=True,
    )
    experience_status_counts_frame.reset_index(inplace=True)

    fig = go.Figure(
        data=[
            go.Table(
                header={"values": experience_status_counts_frame.columns.tolist()},
                cells={
                    "values": experience_status_counts_frame.transpose().values.tolist()
                },
            )
        ]
    )
    resim_plotly_style(fig)
    (
        writer.add_plotly_metric("experience_status_counts")
        .with_description("Experience Status Counts")
        .with_blocking(False)
        .with_should_display(True)
        .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
        .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )


def job_metric_status_counts_metric(
    *,
    writer: ResimMetricsWriter,
    job_metrics_frame: pd.DataFrame,
) -> None:
    job_metric_status_counts_frame = (
        job_metrics_frame.groupby("name")["status"].value_counts().unstack(fill_value=0)
    )

    for status in (
        MetricStatus.FAIL_BLOCK,
        MetricStatus.FAIL_WARN,
        MetricStatus.PASSED,
    ):
        if status not in job_metric_status_counts_frame:
            job_metric_status_counts_frame[status] = 0

    job_metric_status_counts_frame.sort_values(
        by=[
            MetricStatus.FAIL_BLOCK,
            MetricStatus.FAIL_WARN,
            MetricStatus.PASSED,
        ],
        ascending=False,
        inplace=True,
    )
    job_metric_status_counts_frame.reset_index(inplace=True)

    fig = go.Figure(
        data=[
            go.Table(
                header={"values": job_metric_status_counts_frame.columns.tolist()},
                cells={
                    "values": job_metric_status_counts_frame.transpose().values.tolist()
                },
            )
        ]
    )
    resim_plotly_style(fig)
    (
        writer.add_plotly_metric("job_metric_status_counts")
        .with_description("Job Metric Status Counts")
        .with_blocking(False)
        .with_should_display(True)
        .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
        .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )


def batch_metric_status_counts_metric(
    *,
    writer: ResimMetricsWriter,
    batch_metrics_frame: pd.DataFrame,
) -> None:
    batch_metric_status_counts_frame = (
        batch_metrics_frame.groupby("name")["status"]
        .value_counts()
        .unstack(fill_value=0)
    )

    for status in (
        MetricStatus.FAIL_BLOCK,
        MetricStatus.FAIL_WARN,
        MetricStatus.PASSED,
    ):
        if status not in batch_metric_status_counts_frame:
            batch_metric_status_counts_frame[status] = 0

    batch_metric_status_counts_frame.sort_values(
        by=[
            MetricStatus.FAIL_BLOCK,
            MetricStatus.FAIL_WARN,
            MetricStatus.PASSED,
        ],
        ascending=False,
        inplace=True,
    )
    batch_metric_status_counts_frame.reset_index(inplace=True)

    fig = go.Figure(
        data=[
            go.Table(
                header={"values": batch_metric_status_counts_frame.columns.tolist()},
                cells={
                    "values": batch_metric_status_counts_frame.transpose().values.tolist()
                },
            )
        ]
    )
    resim_plotly_style(fig)
    (
        writer.add_plotly_metric("batch_metric_status_counts")
        .with_description("Batch Metric Status Counts")
        .with_blocking(False)
        .with_should_display(True)
        .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
        .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )


def scalar_job_metrics_box_plot_metrics(
    *,
    writer: ResimMetricsWriter,
    tests_frame: pd.DataFrame,
    builds_frame: pd.DataFrame,
    job_metrics_frame: pd.DataFrame,
) -> None:
    scalar_job_metrics_frame = (
        job_metrics_frame[job_metrics_frame.type == MetricType.SCALAR]
        .reset_index()
        .merge(tests_frame["build_id"], on="job_id")
        .merge(builds_frame["build_creation_timestamp"], on="build_id")
    )

    def add_scalar_job_metric_box_plot(df: pd.DataFrame) -> None:
        fig = go.Figure(
            data=[
                go.Box(
                    x=df.build_creation_timestamp,
                    y=df.value,
                    boxpoints="all",
                )
            ]
        )
        name = df["name"].iloc[0]
        unit = df["unit"].iloc[0]
        unitsuffix = f" ({unit})" if unit else ""
        resim_plotly_style(
            fig,
            xaxis_title="Time",
            yaxis_title=name + unitsuffix,
        )

        (
            writer.add_plotly_metric(f"{name}_distribution")
            .with_description(f"{name} distribution over time")
            .with_blocking(False)
            .with_should_display(True)
            .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
            .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
            .with_plotly_data(fig.to_json())
        )

    scalar_job_metrics_frame.groupby("name").apply(add_scalar_job_metric_box_plot)


def scalar_batch_metrics_box_plot_metrics(
    *,
    writer: ResimMetricsWriter,
    batches_frame: pd.DataFrame,
    builds_frame: pd.DataFrame,
    batch_metrics_frame: pd.DataFrame,
) -> None:
    scalar_batch_metrics_frame = (
        batch_metrics_frame[batch_metrics_frame.type == MetricType.SCALAR]
        .reset_index()
        .merge(batches_frame["build_id"], on="batch_id")
        .merge(builds_frame["build_creation_timestamp"], on="build_id")
        .sort_values(by="build_creation_timestamp")
    )

    def add_scalar_batch_metric_line_plot(df: pd.DataFrame) -> None:
        fig = go.Figure(
            data=[
                go.Scatter(
                    x=df.build_creation_timestamp,
                    y=df.value,
                )
            ]
        )
        name = df["name"].iloc[0]
        unit = df["unit"].iloc[0]
        unitsuffix = f" ({unit})" if unit else ""
        resim_plotly_style(
            fig,
            xaxis_title="Time",
            yaxis_title=name + unitsuffix,
        )
        (
            writer.add_plotly_metric(f"{name}_over_time")
            .with_description(f"{name} over time")
            .with_blocking(False)
            .with_should_display(True)
            .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
            .with_status(mu.MetricStatus.NOT_APPLICABLE_METRIC_STATUS)
            .with_plotly_data(fig.to_json())
        )

    scalar_batch_metrics_frame.groupby("name").apply(add_scalar_batch_metric_line_plot)


def compute_metrics(
    *,
    writer: ResimMetricsWriter,
    project_id: uuid.UUID,
    batches_frame: pd.DataFrame,
    tests_frame: pd.DataFrame,
    builds_frame: pd.DataFrame,
    tags_frame: pd.DataFrame,
    job_metrics_frame: pd.DataFrame,
    batch_metrics_frame: pd.DataFrame,
) -> None:
    # pylint: disable=unused-argument
    """Compute the report metrics from batch, tests, builds, tags, and metrics data.

    Args:
        writer:              The writer to write the metrics to.
        project_id:          The project id for the report.
        batches_frame:       Information on the batches in the report.
        tests_frame:         Information on the tests in the report.
        builds_frame:        Information on the builds in the report.
        tags_frame:          Information on the experience and their tags in the report.
        job_metrics_frame:   Information on the job metrics in the report.
        batch_metrics_frame: Information on the batch metrics in the report.
    """

    test_status_over_time_metric(
        writer=writer,
        project_id=project_id,
        tests_frame=tests_frame,
        builds_frame=builds_frame,
    )
    high_level_totals_metrics(
        writer=writer, batches_frame=batches_frame, tests_frame=tests_frame
    )
    experience_status_counts_metric(writer=writer, tests_frame=tests_frame)
    job_metric_status_counts_metric(writer=writer, job_metrics_frame=job_metrics_frame)
    batch_metric_status_counts_metric(
        writer=writer, batch_metrics_frame=batch_metrics_frame
    )

    scalar_job_metrics_box_plot_metrics(
        writer=writer,
        tests_frame=tests_frame,
        builds_frame=builds_frame,
        job_metrics_frame=job_metrics_frame,
    )

    scalar_batch_metrics_box_plot_metrics(
        writer=writer,
        batches_frame=batches_frame,
        builds_frame=builds_frame,
        batch_metrics_frame=batch_metrics_frame,
    )


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

        await fetch_and_compute_metrics(
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
