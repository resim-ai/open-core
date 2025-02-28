# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Test for kickoff_report
"""

import unittest
from datetime import datetime, timezone
from typing import Any
from unittest.mock import patch
from uuid import UUID

from resim_python_client.models import Batch, Report
from resim_python_client.types import UNSET

import resim.orchestration.kickoff_report as kr
import resim.orchestration.resim_python_client_mocks as mocks

GET_BATCH_FUNCTION = "resim.orchestration.kickoff_report.get_batch.asyncio"
GET_TEST_SUITE_FUNCTION = "resim.orchestration.kickoff_report.get_test_suite.asyncio"
CREATE_REPORT_FUNCTION = "resim.orchestration.kickoff_report.create_report.asyncio"


@patch(GET_BATCH_FUNCTION, new=mocks.get_batch_asyncio)
@patch(GET_TEST_SUITE_FUNCTION, new=mocks.get_test_suite_asyncio)
@patch(CREATE_REPORT_FUNCTION, new=mocks.create_report_asyncio)
class KickoffReportTest(unittest.IsolatedAsyncioTestCase):
    async def test_kickoff_report(self) -> None:
        # SETUP
        client = mocks.get_mock_client(mocks.make_mock_state())
        batch: Batch = next(iter(client.state.batches.values()))
        start_timestamp = datetime.now(tz=timezone.utc)

        # ACTION
        report: Report = await kr.run_report_for_batch(
            project_id=UUID(batch.project_id),
            batch_id=UUID(batch.batch_id),
            client=client,
            start_timestamp=start_timestamp,
        )

        # VERIFICATION
        self.assertEqual(report.branch_id, batch.branch_id)
        self.assertEqual(report.metrics_build_id, batch.metrics_build_id)
        self.assertEqual(report.test_suite_id, batch.test_suite_id)
        self.assertEqual(report.start_timestamp, start_timestamp)
        self.assertEqual(report.project_id, batch.project_id)

    async def test_kickoff_report_denied(self) -> None:
        # SETUP
        client = mocks.get_mock_client(mocks.make_mock_state())
        batch = next(iter(client.state.batches.values()))
        start_timestamp = datetime.now(tz=timezone.utc)

        # ACTION
        report = await kr.run_report_for_batch(
            project_id=UUID(batch.project_id),
            batch_id=UUID(batch.batch_id),
            client=client,
            start_timestamp=start_timestamp,
            test_suite_allowlist=set(),
        )

        # VERIFICATION
        self.assertIs(report, None)

    async def test_kickoff_report_unlocalized(self) -> None:
        # SETUP
        client = mocks.get_mock_client(mocks.make_mock_state())
        batch = next(iter(client.state.batches.values()))
        start_timestamp = datetime.now()

        # ACTION / VERIFICATION
        with self.assertRaises(ValueError):
            await kr.run_report_for_batch(
                project_id=UUID(batch.project_id),
                batch_id=UUID(batch.batch_id),
                client=client,
                start_timestamp=start_timestamp,
                test_suite_allowlist=set(),
            )

    async def test_kickoff_report_explicit_metrics_build(self) -> None:
        client = mocks.get_mock_client(mocks.make_mock_state())
        batch: Batch = next(iter(client.state.batches.values()))
        metrics_build_id = [
            k
            for k in client.state.metrics_builds.keys()
            if str(k) != batch.metrics_build_id
        ][0]  # Get another metrics build
        test_suite = client.state.test_suites[UUID(batch.test_suite_id)]
        start_timestamp = datetime.now(tz=timezone.utc)

        # ACTION
        report: Report = await kr.run_report_for_batch(
            project_id=UUID(batch.project_id),
            batch_id=UUID(batch.batch_id),
            client=client,
            start_timestamp=start_timestamp,
            metrics_build_id=metrics_build_id,
            test_suite_allowlist={test_suite.name},
        )

        # VERIFICATION
        self.assertEqual(report.branch_id, batch.branch_id)
        self.assertEqual(report.metrics_build_id, str(metrics_build_id))
        self.assertNotEqual(report.metrics_build_id, batch.metrics_build_id)
        self.assertEqual(report.test_suite_id, batch.test_suite_id)
        self.assertEqual(report.start_timestamp, start_timestamp)
        self.assertEqual(report.project_id, batch.project_id)

    async def test_kickoff_report_no_test_suite(self) -> None:
        # SETUP
        client = mocks.get_mock_client(mocks.make_mock_state())
        batch = next(iter(client.state.batches.values()))
        start_timestamp = datetime.now(tz=timezone.utc)
        test_suite = client.state.test_suites[UUID(batch.test_suite_id)]
        batch.test_suite_id = UNSET

        # ACTION
        report = await kr.run_report_for_batch(
            project_id=UUID(batch.project_id),
            batch_id=UUID(batch.batch_id),
            client=client,
            start_timestamp=start_timestamp,
            test_suite_allowlist={test_suite.name},
        )

        # VERIFICATION
        self.assertIs(report, None)

    async def test_kickoff_report_bad_test_suite_return(self) -> None:
        # SETUP
        client = mocks.get_mock_client(mocks.make_mock_state())
        batch = next(iter(client.state.batches.values()))
        start_timestamp = datetime.now(tz=timezone.utc)
        test_suite = client.state.test_suites[UUID(batch.test_suite_id)]

        async def return_none(*_: Any, **__: Any) -> None:
            return None

        # ACTION / VERIFICATION
        with (
            patch(GET_TEST_SUITE_FUNCTION, new=return_none),
            self.assertRaises(RuntimeError),
        ):
            await kr.run_report_for_batch(
                project_id=UUID(batch.project_id),
                batch_id=UUID(batch.batch_id),
                client=client,
                start_timestamp=start_timestamp,
                test_suite_allowlist={test_suite.name},
            )

    async def test_kickoff_report_bad_create_report(self) -> None:
        # SETUP
        client = mocks.get_mock_client(mocks.make_mock_state())
        batch = next(iter(client.state.batches.values()))
        start_timestamp = datetime.now(tz=timezone.utc)
        test_suite = client.state.test_suites[UUID(batch.test_suite_id)]

        async def return_none(*_: Any, **__: Any) -> None:
            return None

        # ACTION / VERIFICATION
        with (
            patch(CREATE_REPORT_FUNCTION, new=return_none),
            self.assertRaises(RuntimeError),
        ):
            await kr.run_report_for_batch(
                project_id=UUID(batch.project_id),
                batch_id=UUID(batch.batch_id),
                client=client,
                start_timestamp=start_timestamp,
                test_suite_allowlist={test_suite.name},
            )


if __name__ == "__main__":
    unittest.main()
