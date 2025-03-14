# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


import asyncio
import uuid

from resim_python_client.api.batches import list_batches, list_jobs
from resim_python_client.api.reports import get_report
from resim_python_client.client import AuthenticatedClient
from resim_python_client.models.batch import Batch
from resim_python_client.models.job import Job

from resim.metrics.fetch_all_pages import async_fetch_all_pages


async def fetch_batches_for_report(
    *, client: AuthenticatedClient, report_id: uuid.UUID, project_id: uuid.UUID
) -> list[Batch]:
    """Fetch all batches corresponding to a report in the correct time range.

    Note that this does not yet filter batches by test suite revision.
    """
    report_id_str = str(report_id)
    project_id_str = str(project_id)

    report = await get_report.asyncio(project_id_str, report_id_str, client=client)

    if report is None:
        raise RuntimeError("Failed to fetch report")

    test_suite_id = report.test_suite_id
    branch_id = report.branch_id
    start_timestamp = report.start_timestamp
    end_timestamp = report.end_timestamp
    test_suite_revision = report.test_suite_revision
    search_components = [
        f'test_suite_id = "{test_suite_id}"',
        f'branch_id = "{branch_id}"',
        f'created_at > "{start_timestamp}"',
        f'created_at < "{end_timestamp}"',
    ]

    if report.respect_revision_boundary:
        search_components.append(f"test_suite_revision = {test_suite_revision}")

    search_string = " AND ".join(search_components)

    batches: list[Batch] = await async_fetch_all_pages(
        list_batches.asyncio,
        project_id=project_id_str,
        search=search_string,
        client=client,
    )

    batches = [b for page in batches for b in page.batches]
    batches.sort(key=lambda b: b.creation_timestamp)

    return batches


async def fetch_jobs_for_batches(
    *, client: AuthenticatedClient, batch_ids: list[str], project_id: uuid.UUID
) -> dict[str, list[Job]]:
    """Fetch all jobs for the given list of batch_ids and return them in a dict
    of batch ids to lists of jobs."""
    jobs: list[Job] = await asyncio.gather(
        *(
            async_fetch_all_pages(
                list_jobs.asyncio,
                project_id=str(project_id),
                batch_id=batch_id,
                client=client,
            )
            for batch_id in batch_ids
        )
    )
    for i, batch_jobs in enumerate(jobs):
        jobs[i] = [j for page in batch_jobs for j in page.jobs]

    return dict(zip(batch_ids, jobs))
