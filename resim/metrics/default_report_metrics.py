import argparse
import asyncio
import json
import logging
import pathlib
import uuid
from collections import defaultdict

import numpy as np
import pandas as pd
from resim_python_client.client import AuthenticatedClient
from resim_python_client.models.job import Job
from resim_python_client.models.job_status import JobStatus
from resim_python_client.models.metric_status import MetricStatus

from resim.metrics.fetch_report_metrics import (
    fetch_batches_for_report,
    fetch_jobs_for_batches,
)

logger = logging.getLogger("resim")

DEFAULT_CONFIG_PATH = pathlib.Path("/tmp/resim/inputs/report_config.json")


async def main() -> None:
    logging.basicConfig()
    logger.setLevel(logging.INFO)

    parser = argparse.ArgumentParser(
        prog="default_report_metrics",
        description="Compute a basic set of report metrics.",
    )
    parser.add_argument("--report-config", default=str(DEFAULT_CONFIG_PATH))

    args = parser.parse_args()

    with open(args.report_config, "r", encoding="utf-8") as f:
        report_config = json.load(f)

    await compute_metrics(
        token=report_config["authToken"],
        api_url=report_config["apiURL"],
        project_id=uuid.UUID(report_config["projectID"]),
        report_id=uuid.UUID(report_config["reportID"]),
    )


async def compute_metrics(
    *, token: str, api_url: str, project_id: uuid.UUID, report_id: uuid.UUID
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

    status_counts = count_batch_statuses(batch_ids, batch_to_jobs_map)
    print(status_counts)


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


def count_batch_statuses(
    batch_ids: list[str], batch_to_jobs_map: dict[str, list[Job]]
) -> pd.DataFrame:
    status_counts = []
    for batch_id in batch_ids:
        status_counts.append(count_job_statuses(batch_to_jobs_map[batch_id]))

    status_counts = pd.DataFrame(
        data=np.array(status_counts).transpose(),
        index=["PASSED", "FAIL_WARN", "FAIL_BLOCK", "ERROR", "CANCELLED", "UNKNOWN"],
        columns=range(len(batch_ids)),
    )

    return status_counts


if __name__ == "__main__":
    asyncio.run(main())
