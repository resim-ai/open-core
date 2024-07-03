import datetime
import unittest
import uuid
from dataclasses import dataclass
from unittest.mock import patch

from resim_python_client.api.batches import list_batches
from resim_python_client.models.batch import Batch
from resim_python_client.models.list_batches_output import ListBatchesOutput
from resim_python_client.models.metric_status import MetricStatus
from resim_python_client.models.report import Report
from resim_python_client.models.report_status import ReportStatus

import resim.metrics.fetch_report_metrics as frp


@dataclass(frozen=True)
class ClientMock:
    name: str = "ClientMock"


class FetchReportMetricsTest(unittest.IsolatedAsyncioTestCase):
    def __init__(self, *args, **kwargs):
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
            respect_revision_boundary=False,
            start_timestamp=datetime.datetime(2022, 1, 30),
            status=ReportStatus.SUCCEEDED,
            status_history=[],
            test_suite_id=str(uuid.uuid4()),
            test_suite_revision=0,
            user_id="",
        )
        self.client = ClientMock()

        self.batches = [
            ListBatchesOutput(
                batches=[Batch(batch_id=str(uuid.uuid4()), associated_account="")]
            ),
            ListBatchesOutput(
                batches=[Batch(batch_id=str(uuid.uuid4()), associated_account="")]
            ),
        ]

    @patch("resim.metrics.fetch_report_metrics.AuthenticatedClient", ClientMock)
    @patch("resim.metrics.fetch_report_metrics.get_report.asyncio")
    @patch("resim.metrics.fetch_report_metrics.async_fetch_all_pages")
    async def test_fetch_batches_for_report(self, get_batches_mock, get_report_mock):
        get_report_mock.return_value = self.report
        get_batches_mock.return_value = self.batches

        batches = await frp.fetch_batches_for_report(
            client=self.client,
            report_id=uuid.UUID(self.report.report_id),
            project_id=uuid.UUID(self.report.project_id),
        )

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

        search_components = get_batches_mock.call_args.kwargs["search"].split(" AND ")
        self.assertIn(
            f'test_suite_id = "{self.report.test_suite_id}"', search_components
        )
        self.assertIn(f'branch_id = "{self.report.branch_id}"', search_components)
        self.assertIn(
            f'created_at > "{self.report.start_timestamp}"', search_components
        )
        self.assertIn(f'created_at < "{self.report.end_timestamp}"', search_components)

        self.assertEqual(batches[0].batch_id, self.batches[0].batches[0].batch_id)
        self.assertEqual(batches[1].batch_id, self.batches[1].batches[0].batch_id)

        get_report_mock.return_value = None

        with self.assertRaises(RuntimeError):
            await frp.fetch_batches_for_report(
                client=self.client,
                report_id=uuid.UUID(self.report.report_id),
                project_id=uuid.UUID(self.report.project_id),
            )


if __name__ == "__main__":
    unittest.main()
