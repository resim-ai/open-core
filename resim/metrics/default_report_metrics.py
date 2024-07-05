import argparse
import asyncio
import json
import logging
import pathlib
import uuid
from collections import defaultdict

import numpy as np
import pandas as pd
import plotly.express as px
from resim_python_client.client import AuthenticatedClient
from resim_python_client.models.job import Job
from resim_python_client.models.job_status import JobStatus
from resim_python_client.models.metric_status import MetricStatus

import resim.metrics.python.metrics_utils as mu
from resim.metrics.fetch_report_metrics import (
    fetch_batches_for_report,
    fetch_jobs_for_batches,
)
from resim.metrics.proto.validate_metrics_proto import validate_job_metrics
from resim.metrics.python.metrics_writer import ResimMetricsWriter

logger = logging.getLogger("resim")

DEFAULT_CONFIG_PATH = pathlib.Path("/tmp/resim/inputs/report_config.json")
DEFAULT_OUTPUT_PATH = pathlib.Path("/tmp/resim/outputs/metrics.binproto")


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


async def compute_metrics(
    *,
    token: str,
    api_url: str,
    project_id: uuid.UUID,
    report_id: uuid.UUID,
    output_path: str
) -> None:
    client = AuthenticatedClient(base_url=api_url, token=token)

    batches = await fetch_batches_for_report(
        client=client, report_id=report_id, project_id=project_id
    )

    logger.info("Fetched %d batches for report.", len(batches))

    if len(batches) == 0:
        return

    batch_ids = [b.batch_id for b in batches]
    batch_to_jobs_map = await fetch_jobs_for_batches(
        client=client, batch_ids=batch_ids, project_id=project_id
    )

    for batch_id, jobs in batch_to_jobs_map.items():
        logger.info("Fetched %d jobs for batch %s", len(jobs), batch_id)

    status_counts = count_batch_statuses(batch_ids, batch_to_jobs_map)
    print(status_counts)

    fig = px.area(
        status_counts,
        x=range(len(batch_ids)),
        y=["PASSED", "FAIL_WARN", "FAIL_BLOCK", "ERROR", "CANCELLED", "UNKNOWN"],
    )

    writer = ResimMetricsWriter(uuid.uuid4())  # Make metrics writer!
    (
        writer.add_plotly_metric("Plotly example")
        .with_description("Some sort of thing.")
        .with_blocking(False)
        .with_should_display(True)
        .with_importance(mu.MetricImportance.HIGH_IMPORTANCE)
        .with_status(mu.MetricStatus.PASSED_METRIC_STATUS)
        .with_plotly_data(fig.to_json())
    )
    metrics_proto = writer.write()
    with open(output_path, "wb") as f:
        f.write(metrics_proto.metrics_msg.SerializeToString())


def count_batch_statuses(
    batch_ids: list[str], batch_to_jobs_map: dict[str, list[Job]]
) -> pd.DataFrame:
    status_counts = []
    for batch_id in batch_ids:
        status_counts.append(count_job_statuses(batch_to_jobs_map[batch_id]))

    status_counts = pd.DataFrame(
        data=np.array(status_counts),
        columns=["PASSED", "FAIL_WARN", "FAIL_BLOCK", "ERROR", "CANCELLED", "UNKNOWN"],
        index=range(len(batch_ids)),
    )

    return status_counts


def count_job_statuses(jobs: list[Job]) -> list[int]:
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
