import asyncio
from resim_python_client.client import AuthenticatedClient
from resim.auth.python.device_code_client import DeviceCodeClient
from resim_python_client.api.batches import list_jobs, list_metrics_for_job
from uuid import UUID
import pandas as pd
import argparse
import asyncio

from resim_python_client.models import Job, MetricType
from resim.metrics.fetch_all_pages import async_fetch_all_pages
from resim_python_client.api.parameter_sweeps import get_parameter_sweep


async def _fetch_jobs_for_batch(
    *, client: AuthenticatedClient, batch_id: UUID, project_id: UUID
) -> list[Job]:
    job_pages = await async_fetch_all_pages(
        list_jobs.asyncio,
        client=client,
        project_id=str(project_id),
        batch_id=str(batch_id),
    )
    return [j for page in job_pages for j in page.jobs]


async def _fetch_scalar_metrics_for_job(
    *, client: AuthenticatedClient, job_id: UUID, batch_id: UUID, project_id: UUID
) -> pd.DataFrame:
    job_metrics_pages = await async_fetch_all_pages(
        list_metrics_for_job.asyncio,
        client=client,
        project_id=str(project_id),
        batch_id=str(batch_id),
        job_id=str(job_id),
    )
    return [
        (m.name, m.value, m.unit)
        for page in job_metrics_pages
        for m in page.metrics
        if m.type == MetricType.SCALAR
    ]


async def fetch_sweep_metrics_frame(
    *, client: AuthenticatedClient, sweep_id: UUID, project_id: UUID
) -> pd.DataFrame:
    sweep = await get_parameter_sweep.asyncio(
        project_id=str(project_id), sweep_id=str(sweep_id), client=client
    )
    batch_ids = sweep.batches

    jobs = [
        await _fetch_jobs_for_batch(
            client=client, batch_id=UUID(bid), project_id=project_id
        )
        for bid in batch_ids
    ]

    # Could be done more async with a task group
    jobs_frame = pd.DataFrame(
        [(j.job_id, j.batch_id) for batch in jobs for j in batch],
        columns=("job_id", "batch_id"),
    )

    sem = asyncio.Semaphore(20)

    async def fetch_metrics(row: tuple):
        async with sem:
            job_id = row["job_id"]
            batch_id = row["batch_id"]
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
            )

    frames = await asyncio.gather(
        *(fetch_metrics(row) for _, row in jobs_frame.iterrows())
    )

    metrics_frame = pd.concat(frames, ignore_index=True)
    metrics_frame.to_csv("/home/ubuntu/job_metrics.csv", index=False)


async def main():
    parser = argparse.ArgumentParser(prog="fetch_sweep_metrics")
    parser.add_argument("--project-id", type=UUID, required=True)
    parser.add_argument("--sweep-id", type=UUID, required=True)

    args = parser.parse_args()
    device_code_client = DeviceCodeClient()
    client = AuthenticatedClient(
        base_url="https://api.resim.ai/v1/",
        token=device_code_client.get_jwt()["access_token"],
    )

    await fetch_sweep_metrics_frame(
        client=client,
        sweep_id=args.sweep_id,
        project_id=args.project_id,
    )


if __name__ == "__main__":
    asyncio.run(main())
