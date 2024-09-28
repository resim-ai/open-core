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
    batches_frame: pd.DataFrame,
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
                x=status_counts_frame.build_creation_timestamp,
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
                header=dict(values=["Name", "Value"]),
                cells=dict(
                    values=[
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
                ),
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
                header=dict(values=list(experience_status_counts_frame.columns)),
                cells=dict(
                    values=experience_status_counts_frame.transpose().values.tolist()
                ),
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
                header=dict(values=list(job_metric_status_counts_frame.columns)),
                cells=dict(
                    values=job_metric_status_counts_frame.transpose().values.tolist()
                ),
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
                header=dict(values=list(batch_metric_status_counts_frame.columns)),
                cells=dict(
                    values=batch_metric_status_counts_frame.transpose().values.tolist()
                ),
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

    scalar_job_metrics_frame = (
        job_metrics_frame[job_metrics_frame.type == MetricType.SCALAR]
        .reset_index()
        .merge(tests_frame["build_id"], on="job_id")
        .merge(builds_frame["build_creation_timestamp"], on="build_id")
    )

    def add_scalar_job_metric_histogram(df: pd.DataFrame):

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

    scalar_job_metrics_frame.groupby("name").apply(add_scalar_job_metric_histogram)

    scalar_batch_metrics_frame = (
        batch_metrics_frame[batch_metrics_frame.type == MetricType.SCALAR]
        .reset_index()
        .merge(batches_frame["build_id"], on="batch_id")
        .merge(builds_frame["build_creation_timestamp"], on="build_id")
        .sort_values(by="build_creation_timestamp")
    )

    def add_scalar_batch_metric_histogram(df: pd.DataFrame):
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

    scalar_batch_metrics_frame.groupby("name").apply(add_scalar_batch_metric_histogram)


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

    def batch_conflated_status(batch: pd.DataFrame):
        if batch.batch_status in (
            BatchStatus.BATCH_METRICS_QUEUED,
            BatchStatus.BATCH_METRICS_RUNNING,
            BatchStatus.EXPERIENCES_RUNNING,
        ):
            return ConflatedJobStatus.RUNNING
        if batch.batch_status == BatchStatus.CANCELLED:
            return ConflatedJobStatus.CANCELLED
        if batch.batch_status == BatchStatus.ERROR:
            return ConflatedJobStatus.ERROR
        if batch.batch_status == BatchStatus.SUBMITTED:
            return ConflatedJobStatus.QUEUED
        if batch.batch_status == BatchStatus.SUCCEEDED:
            if (
                MetricStatus.FAIL_BLOCK
                in batch[["batch_metrics_status", "jobs_metrics_status"]].to_list()
            ):
                return ConflatedJobStatus.BLOCKER
            elif (
                MetricStatus.FAIL_WARN
                in batch[["batch_metrics_status", "jobs_metrics_status"]].to_list()
            ):
                return ConflatedJobStatus.WARNING
            return ConflatedJobStatus.PASSED

    batches_frame["status"] = batches_frame.apply(batch_conflated_status, axis=1)

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
        batches_frame=batches_frame,
        tests_frame=tests_frame,
        builds_frame=builds_frame,
        tags_frame=tags_frame,
        job_metrics_frame=job_metrics_frame,
        batch_metrics_frame=batch_metrics_frame,
    )

    #    all_metrics = [
    #        job_status_categories_metric,
    #        flaky_experiences_metric,
    #        batch_metrics_scalars_over_time_metric,
    #    ]
    #
    #    for metric in all_metrics:
    #        metric(
    #            writer,
    #            batches=batches,
    #            batch_to_jobs_map=batch_to_jobs_map,
    #            job_to_metrics_map=job_to_metrics_map,
    #            batch_to_metrics_map=batch_to_metrics_map,
    #            scalar_batch_metrics_map=scalar_batch_metrics_map,
    #        )

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
