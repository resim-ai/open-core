# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import datetime
import unittest
import uuid
from dataclasses import dataclass
from typing import Any
from unittest.mock import Mock, patch

from resim_python_client.api.batches import list_batches, list_jobs
from resim_python_client.models.batch import Batch
from resim_python_client.models.list_batches_output import ListBatchesOutput
from resim_python_client.models.list_jobs_output import ListJobsOutput
from resim_python_client.models.metric_status import MetricStatus
from resim_python_client.models.report import Report
from resim_python_client.models.report_status import ReportStatus

import resim.metrics.fetch_report_metrics as frp


@dataclass(frozen=True)
class ClientMock:
    name: str = "ClientMock"


class FetchReportMetricsTest(unittest.IsolatedAsyncioTestCase):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.report = Report(
            associated_account="",
            branch_id=str(uuid.uuid4()),
            creation_timestamp=datetime.datetime(2023, 1, 30),
            end_timestamp=datetime.datetime(2021, 10, 31),
            last_updated_timestamp=datetime.datetime(2023, 2, 1),
            metrics_build_id="",
            metrics_status=MetricStatus.PASSED,
            name="my test report",
            org_id="",
            output_location="",
            project_id=str(uuid.uuid4()),
            report_id=str(uuid.uuid4()),
            respect_revision_boundary=True,
            start_timestamp=datetime.datetime(2022, 1, 30),
            status=ReportStatus.SUCCEEDED,
            status_history=[],
            test_suite_id=str(uuid.uuid4()),
            test_suite_revision=3,
            user_id="",
            metrics_set_name="blah",
        )
        self.client = ClientMock()

        self.batches = [
            ListBatchesOutput(
                batches=[
                    Batch(
                        batch_id=str(uuid.uuid4()),
                        creation_timestamp=datetime.datetime(2022, 2, 1),
                        associated_account="",
                    )
                ]
            ),
            ListBatchesOutput(
                batches=[
                    Batch(
                        batch_id=str(uuid.uuid4()),
                        creation_timestamp=datetime.datetime(2022, 2, 2),
                        associated_account="",
                    )
                ]
            ),
        ]

        self.batches_to_jobs = {
            self.batches[0].batches[0].batch_id: ListJobsOutput(jobs=[]),
            self.batches[1].batches[0].batch_id: ListJobsOutput(jobs=[]),
        }

    @patch("resim.metrics.fetch_report_metrics.AuthenticatedClient", ClientMock)
    @patch("resim.metrics.fetch_report_metrics.get_report.asyncio")
    @patch("resim.metrics.fetch_report_metrics.async_fetch_all_pages")
    async def test_fetch_batches_for_report(
        self, get_batches_mock: Mock, get_report_mock: Mock
    ) -> None:
        for respect_revision_boundary in (True, False):
            self.report.respect_revision_boundary = respect_revision_boundary

            # SETUP
            get_report_mock.return_value = self.report
            get_batches_mock.return_value = self.batches

            # ACTION
            batches = await frp.fetch_batches_for_report(
                client=self.client,
                report_id=uuid.UUID(self.report.report_id),
                project_id=uuid.UUID(self.report.project_id),
            )

            # VERIFICATION
            get_report_mock.assert_called_once()
            self.assertEqual(get_report_mock.call_args.args[0], self.report.project_id)
            self.assertEqual(get_report_mock.call_args.args[1], self.report.report_id)
            self.assertEqual(get_report_mock.call_args.kwargs["client"], self.client)

            get_batches_mock.assert_called_once()
            self.assertEqual(get_batches_mock.call_args.args[0], list_batches.asyncio)
            self.assertEqual(get_batches_mock.call_args.kwargs["client"], self.client)
            self.assertEqual(
                get_batches_mock.call_args.kwargs["project_id"], self.report.project_id
            )

            search_components = get_batches_mock.call_args.kwargs["search"].split(
                " AND "
            )
            NUM_SEARCH_COMPONENTS = 5 if respect_revision_boundary else 4
            self.assertEqual(len(search_components), NUM_SEARCH_COMPONENTS)
            self.assertIn(
                f'test_suite_id = "{self.report.test_suite_id}"', search_components
            )
            self.assertIn(f'branch_id = "{self.report.branch_id}"', search_components)
            self.assertIn(
                f'created_at > "{self.report.start_timestamp}"', search_components
            )
            self.assertIn(
                f'created_at < "{self.report.end_timestamp}"', search_components
            )
            if respect_revision_boundary:
                self.assertIn(
                    f"test_suite_revision = {self.report.test_suite_revision}",
                    search_components,
                )

            self.assertEqual(batches[0].batch_id, self.batches[0].batches[0].batch_id)
            self.assertEqual(batches[1].batch_id, self.batches[1].batches[0].batch_id)

            # SETUP
            get_report_mock.return_value = None

            # ACTION / VERIFICATION
            with self.assertRaises(RuntimeError):
                await frp.fetch_batches_for_report(
                    client=self.client,
                    report_id=uuid.UUID(self.report.report_id),
                    project_id=uuid.UUID(self.report.project_id),
                )

            get_batches_mock.reset_mock()
            get_report_mock.reset_mock()

    @patch("resim.metrics.fetch_report_metrics.AuthenticatedClient", ClientMock)
    @patch("resim.metrics.fetch_report_metrics.async_fetch_all_pages")
    async def test_fetch_jobs_for_batches(self, list_jobs_mock: Mock) -> None:
        # SETUP
        def list_jobs_mock_impl(
            *_: Any, batch_id: str = "", **__: Any
        ) -> list[ListBatchesOutput]:
            return [self.batches_to_jobs[batch_id]]

        list_jobs_mock.side_effect = list_jobs_mock_impl

        # ACTION
        batch_ids = [b.batch_id for page in self.batches for b in page.batches]
        batch_ids.sort()
        batches_to_jobs = await frp.fetch_jobs_for_batches(
            client=self.client,
            batch_ids=batch_ids,
            project_id=uuid.UUID(self.report.project_id),
        )

        # VERIFICATION
        observed_batch_ids = []
        for call_args in list_jobs_mock.call_args_list:
            self.assertEqual(call_args.args[0], list_jobs.asyncio)
            self.assertEqual(call_args.kwargs["project_id"], self.report.project_id)
            self.assertEqual(call_args.kwargs["client"], self.client)
            observed_batch_ids.append(call_args.kwargs["batch_id"])
        observed_batch_ids.sort()

        self.assertEqual(batch_ids, observed_batch_ids)

        self.assertEqual(len(batch_ids), len(batches_to_jobs))
        for batch_id in batch_ids:
            self.assertIn(batch_id, batches_to_jobs)
            self.assertEqual(
                batches_to_jobs[batch_id], self.batches_to_jobs[batch_id].jobs
            )


if __name__ == "__main__":
    unittest.main()
