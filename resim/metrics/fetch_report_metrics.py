
import asyncio
import uuid

from resim_python_client.client import AuthenticatedClient
from resim_python_client.models.batch import Batch
from resim_python_client.models.job import Job
from resim_python_client.api.reports import get_report
from resim_python_client.api.batches import list_batches, list_jobs
from resim.metrics.fetch_all_pages import async_fetch_all_pages



async def fetch_batches_for_report(*,
    client: AuthenticatedClient,
    report_id: uuid.UUID,
    project_id: uuid.UUID) -> list[Batch]:
    report_id = str(report_id)
    project_id = str(project_id)

    report = await get_report.asyncio(project_id, report_id, client=client)

    if report is None:
        raise ValueError("Failed to fetch report")

    test_suite_id = report.test_suite_id
    branch_id = report.branch_id
    start_timestamp = report.start_timestamp
    end_timestamp = report.end_timestamp
    search_string = f"test_suite_id = \"{test_suite_id}\" AND branch_id = \"{branch_id}\" AND created_at > \"{start_timestamp}\" AND created_at < \"{end_timestamp}\""

    batches = await async_fetch_all_pages(list_batches.asyncio, project_id=project_id, search=search_string, client=client)

    batches = [b for page in batches for b in page.batches]
    return batches


async def fetch_jobs_for_batches(*,
    client: AuthenticatedClient,
    batch_ids: list[str],
    project_id: uuid.UUID) -> dict[str, list[Job]]:
    jobs = await asyncio.gather(*(async_fetch_all_pages(list_jobs.asyncio,
                                              project_id=project_id,
                                              batch_id=batch_id,
                                              client=client) for batch_id in batch_ids))
    for i, batch_jobs in enumerate(jobs):
        jobs[i] = [j for page in batch_jobs for j in page.jobs]

    return {batch_id: batch_jobs for batch_id, batch_jobs in zip(batch_ids, jobs)}
