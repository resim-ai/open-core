# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import logging
import uuid
from datetime import datetime
from typing import Any, Optional

from resim_python_client.api.batches import get_batch
from resim_python_client.api.reports import create_report
from resim_python_client.api.test_suites import get_test_suite
from resim_python_client.client import AuthenticatedClient
from resim_python_client.models import Report, ReportInput

logger = logging.getLogger(__name__)


async def run_report_for_batch(
    *,
    project_id: uuid.UUID,
    batch_id: uuid.UUID,
    client: AuthenticatedClient,
    start_timestamp: datetime,
    test_suite_allowlist: Optional[set[str]] = None,
    metrics_build_id: Optional[uuid.UUID] = None,
    **kwargs: Any,
) -> Optional[Report]:
    """Run a report based on a given batch_id and start_timetamp.

    Run a report using the same metrics build as the given batch, or one optionally provided. Also,
    allow for a specific set of allowed test suite names to be provided (e.g. nightly test
    suites). This function is useful for triggering a report as the final step of a batch on the
    same test suite to ensure that the current batch gets included in the report.

    Args:
      project_id - The ID of this project,
      batch_id - The ID of the batch to get the test suite and metrics build id from.
      client - The client to use when querying the API.
      start_timestamp - The time to start the report from.
      test_suite_allowlist - The test suites to allow reports to be spawned for. If omitted, all are
                             allowed
      metrics_build_id - An optional metrics build override to use for the report.
      kwargs - Keyword arguments forwarded to create_report.asyncio
    """
    if start_timestamp.tzinfo is None:
        raise ValueError("start_timestamp must be localized!")
    batch = await get_batch.asyncio(
        batch_id=str(batch_id), project_id=str(project_id), client=client
    )
    test_suite_id = batch.test_suite_id
    if not test_suite_id:
        return None
    branch_id = batch.branch_id
    if metrics_build_id is None:
        metrics_build_id = uuid.UUID(batch.metrics_build_id)

    test_suite = await get_test_suite.asyncio(
        project_id=str(project_id), test_suite_id=test_suite_id, client=client
    )
    if test_suite is None:
        raise RuntimeError("Could not find test suite")

    if test_suite_allowlist is not None and test_suite.name not in test_suite_allowlist:
        return None

    report = await create_report.asyncio(
        client=client,
        project_id=str(project_id),
        body=ReportInput(
            branch_id=branch_id,
            metrics_build_id=str(metrics_build_id),
            start_timestamp=start_timestamp,
            test_suite_id=test_suite_id,
            **kwargs,
        ),
    )
    if report is None:
        raise RuntimeError("Failed to create report")

    logger.info("Created report: %s", report.report_id)
    return report
