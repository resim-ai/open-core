# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Test for resim_python_client_mocks
"""

import unittest
from datetime import datetime
from uuid import UUID, uuid4

from resim_python_client.models import Batch, Report, ReportInput, TestSuite

import resim.orchestration.resim_python_client_mocks as mocks


def make_mock_state() -> mocks.MockState:
    """Helper to create a MockState containing what we need for this test."""
    state = mocks.MockState()

    for _ in range(5):
        p = mocks.random_project(state)
        state.projects[UUID(p.project_id)] = p

    for _ in range(5):
        mb = mocks.random_metrics_build(state)
        state.metrics_builds[UUID(mb.metrics_build_id)] = mb

    for _ in range(5):
        ts = mocks.random_test_suite(state)
        state.test_suites[UUID(ts.test_suite_id)] = ts

    for _ in range(5):
        b = mocks.random_batch(state)
        state.batches[UUID(b.batch_id)] = b

    return state


class ReSimPythonClientMocksTest(unittest.IsolatedAsyncioTestCase):
    async def test_get_batch(self) -> None:
        # SETUP
        client = mocks.get_mock_client(make_mock_state())
        batch: Batch = next(iter(client.state.batches.values()))

        # ACTION
        received_batch = await mocks.get_batch_asyncio(
            project_id=batch.project_id, batch_id=batch.batch_id, client=client
        )

        # VERIFICATION
        self.assertIs(batch, received_batch)

        # ACTION / VERIFICATION
        with self.assertRaises(ValueError):
            await mocks.get_batch_asyncio(
                project_id=str(uuid4()), batch_id=batch.batch_id, client=client
            )

    async def test_get_test_suite(self) -> None:
        # SETUP
        client = mocks.get_mock_client(make_mock_state())
        test_suite: TestSuite = next(iter(client.state.test_suites.values()))

        # ACTION
        received_test_suite = await mocks.get_test_suite_asyncio(
            project_id=test_suite.project_id,
            test_suite_id=test_suite.test_suite_id,
            client=client,
        )

        # VERIFICATION
        self.assertIs(test_suite, received_test_suite)

        # ACTION / VERIFICATION
        with self.assertRaises(ValueError):
            await mocks.get_test_suite_asyncio(
                project_id=str(uuid4()),
                test_suite_id=test_suite.test_suite_id,
                client=client,
            )

    async def test_create_report(self) -> None:
        # SETUP
        client = mocks.get_mock_client(make_mock_state())
        batch: Batch = next(iter(client.state.batches.values()))
        start_timestamp = datetime.now()

        body = ReportInput(
            branch_id=batch.branch_id,
            metrics_build_id=batch.metrics_build_id,
            start_timestamp=start_timestamp,
            test_suite_id=batch.test_suite_id,
        )

        # ACTION
        report: Report = await mocks.create_report_asyncio(
            project_id=batch.project_id,
            body=body,
            client=client,
        )

        # VERIFICATION
        self.assertIs(report, client.state.reports[UUID(report.report_id)])
        self.assertEqual(report.branch_id, batch.branch_id)
        self.assertEqual(report.metrics_build_id, batch.metrics_build_id)
        self.assertEqual(report.start_timestamp, start_timestamp)
        self.assertEqual(report.test_suite_id, batch.test_suite_id)


if __name__ == "__main__":
    unittest.main()
