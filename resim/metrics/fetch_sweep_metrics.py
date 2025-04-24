# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import asyncio
from resim_python_client.client import AuthenticatedClient
from resim.auth.python.device_code_client import DeviceCodeClient
from resim_python_client.api.batches import list_jobs, list_metrics_for_job
from uuid import UUID
import pandas as pd
import argparse
from pathlib import Path

from resim_python_client.models import (
    Job,
    MetricType,
    ListJobsOutput,
    ListJobMetricsOutput,
)
from resim.metrics.fetch_all_pages import async_fetch_all_pages
from resim_python_client.api.parameter_sweeps import get_parameter_sweep

_PAGE_SIZE = 100


async def _fetch_jobs_for_batch(
    *, client: AuthenticatedClient, batch_id: UUID, project_id: UUID
) -> list[Job]:
    """
    Fetch all the jobs corresponding to a single batch.
    """
    job_pages: list[ListJobsOutput] = await async_fetch_all_pages(
        list_jobs.asyncio,
        client=client,
        project_id=str(project_id),
        batch_id=str(batch_id),
        page_size=_PAGE_SIZE,
    )
    return [j for page in job_pages for j in page.jobs]


async def _fetch_scalar_metrics_for_job(
    *, client: AuthenticatedClient, job_id: UUID, batch_id: UUID, project_id: UUID
) -> list[tuple]:
    """
    Fetch all scalar metrics for a single job.
    """

    job_metrics_pages: ListJobMetricsOutput = await async_fetch_all_pages(
        list_metrics_for_job.asyncio,
        client=client,
        project_id=str(project_id),
        batch_id=str(batch_id),
        job_id=str(job_id),
        page_size=_PAGE_SIZE,
    )
    return [
        (m.name, m.value, m.unit)
        for page in job_metrics_pages
        for m in page.metrics
        if m.type == MetricType.SCALAR
    ]


async def fetch_sweep_metrics_frame(
    *, client: AuthenticatedClient, sweep_id: UUID, project_id: UUID, pivot: bool
) -> pd.DataFrame:
    """
    Compute the sweep metrics frame.

    There are two options, if pivot==false, we just produce a single frame with columns:
    (batch_id, job_id, sweep_params..., name, value, unit)
    where each batch id and job id is repeated once for each metric in that job.

    If pivot==true, we produce a single frame with columns:
    (batch_id, job_id, sweep_params..., metrics_values...)

    where each job now appears only once and units are no longer present.
    """
    sweep = await get_parameter_sweep.asyncio(
        project_id=str(project_id), sweep_id=str(sweep_id), client=client
    )
    batch_ids = sweep.batches

    jobs_per_batch = [
        await _fetch_jobs_for_batch(
            client=client, batch_id=UUID(bid), project_id=project_id
        )
        for bid in batch_ids
    ]
    jobs = [j for batch in jobs_per_batch for j in batch]

    # Could be done more async with a task group
    job_rows = []
    param_set = set()
    for j in jobs:
        param_set.update(j.parameters.additional_properties)

    params = list(param_set)
    for j in jobs:
        job_rows.append(
            (j.job_id, j.batch_id)
            + tuple((j.parameters.additional_properties[p] for p in params))
        )

    columns = ("job_id", "batch_id") + tuple(params)

    jobs_frame = pd.DataFrame(job_rows, columns=columns).set_index(
        ["batch_id", "job_id"]
    )

    # Use a semaphore so we don't try to fetch too many jobs' metrics simultaneously
    sem = asyncio.Semaphore(20)

    async def fetch_metrics(idx: pd.DataFrame) -> pd.DataFrame:
        async with sem:
            batch_id, job_id = idx
            metrics = await _fetch_scalar_metrics_for_job(
                client=client,
                job_id=UUID(job_id),
                batch_id=UUID(batch_id),
                project_id=project_id,
            )
            return pd.DataFrame(
                [
                    (batch_id, job_id, name, value, unit)
                    for (name, value, unit) in metrics
                ],
                columns=("batch_id", "job_id", "name", "value", "unit"),
            ).set_index(["batch_id", "job_id"])

    frames = await asyncio.gather(
        *(fetch_metrics(index) for index, _ in jobs_frame.iterrows())
    )
    metrics_frame = pd.concat(frames)

    if pivot:
        metrics_frame = metrics_frame.pivot(columns="name", values="value")
    return jobs_frame.join(metrics_frame).reset_index()


async def main() -> None:
    parser = argparse.ArgumentParser(
        prog="fetch_sweep_metrics",
        description="""
This program computes a DataFrame containing all *scalar* job metrics data from a parameter sweep.
    
There are two options for how this DataFrame can be laid out:

If pivot==false, we just produce a single frame with columns:
(batch_id, job_id, sweep_params..., name, value, unit)
where each batch id and job id is repeated once for each metric in that job.

If pivot==true, we produce a single frame with columns:
(batch_id, job_id, sweep_params..., metrics_values...)
where each job now appears only once and units are no longer present. Note that
null entries may result in the case where some jobs produce different metrics
from others.
""",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--project-id", type=UUID, required=True)
    parser.add_argument("--sweep-id", type=UUID, required=True)
    parser.add_argument(
        "--pivot",
        action="store_true",
        help="Pivot the resulting metrics so that each metric gets its own column",
    )
    parser.add_argument("--output-path", type=Path, required=True)

    args = parser.parse_args()
    device_code_client = DeviceCodeClient()
    client = AuthenticatedClient(
        base_url="https://api.resim.ai/v1/",
        token=device_code_client.get_jwt()["access_token"],
    )

    metrics_frame = await fetch_sweep_metrics_frame(
        client=client,
        sweep_id=args.sweep_id,
        project_id=args.project_id,
        pivot=args.pivot,
    )

    metrics_frame.to_csv(args.output_path, index=False)


if __name__ == "__main__":
    asyncio.run(main())
