# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Mocks for the resim python client so we don't have to hit the actual API.
"""

import random
from dataclasses import dataclass, field
from datetime import datetime, timezone
from inspect import Signature, signature
from string import ascii_lowercase
from typing import Any, Callable, Optional, TypeVar, Union
from unittest.mock import MagicMock
from uuid import UUID, uuid4

from resim_python_client.api.batches import get_batch
from resim_python_client.api.reports import create_report
from resim_python_client.api.test_suites import get_test_suite
from resim_python_client.client import AuthenticatedClient
from resim_python_client.models import (
    Batch,
    MetricsBuild,
    MetricStatus,
    Project,
    Report,
    ReportInput,
    ReportStatus,
    TestSuite,
)


@dataclass
class MockState:
    """The state of the mock (i.e. mocking the internal state of the app)."""

    org_id: str = "simbox.tech"
    projects: dict[UUID, Project] = field(default_factory=dict)
    metrics_builds: dict[UUID, MetricsBuild] = field(default_factory=dict)
    batches: dict[UUID, Batch] = field(default_factory=dict)
    test_suites: dict[UUID, TestSuite] = field(default_factory=dict)
    reports: dict[UUID, Report] = field(default_factory=dict)


K = TypeVar("K")
V = TypeVar("V")


def _random_dict_value(d: dict[K, V]) -> V:
    k: K = random.choice(list(d.keys()))
    return d[k]


def _random_bool() -> bool:
    return random.choice([True, False])


def _random_string() -> str:
    return str(random.choice(ascii_lowercase) for _ in range(20))


def random_project(state: MockState) -> Project:
    """Get a random project consistent with the current state."""
    project_id = uuid4()
    return Project(
        creation_timestamp=datetime.now(tz=timezone.utc),
        description=_random_string(),
        name=_random_string(),
        org_id=state.org_id,
        project_id=str(project_id),
        user_id=f"{_random_string()}@{state.org_id}",
        archived=False,
    )


def random_batch(state: MockState) -> Batch:
    """Get a random batch consistent with the current state."""
    batch_id = uuid4()
    branch_id = uuid4()  # TODO(michael) Once we add builds, get this from there.
    test_suite = _random_dict_value(state.test_suites)
    return Batch(
        associated_account="",
        project_id=test_suite.project_id,
        batch_id=str(batch_id),
        org_id=state.org_id,
        branch_id=str(branch_id),
        test_suite_id=test_suite.test_suite_id,
        metrics_build_id=test_suite.metrics_build_id,
    )


def random_test_suite(state: MockState) -> Batch:
    """Get a random test suite consistent with the current state."""
    test_suite_id = uuid4()
    metrics_build = _random_dict_value(state.metrics_builds)
    return TestSuite(
        creation_timestamp=datetime.now(tz=timezone.utc),
        description=_random_string(),
        experiences=[],
        name=f"test suite {test_suite_id}",
        org_id=state.org_id,
        project_id=metrics_build.project_id,
        system_id=str(uuid4()),
        test_suite_id=str(test_suite_id),
        test_suite_revision=random.randint(1, 100),
        user_id=f"{_random_string()}@{state.org_id}",
        metrics_build_id=metrics_build.metrics_build_id,
        show_on_summary=_random_bool(),
    )


def random_metrics_build(state: MockState) -> MetricsBuild:
    """Get a random metrics build consistent with the current state."""
    project = _random_dict_value(state.projects)
    metrics_build_id = uuid4()
    return MetricsBuild(
        creation_timestamp=datetime.now(tz=timezone.utc),
        image_uri=_random_string(),
        metrics_build_id=str(metrics_build_id),
        name=f"metrics build {metrics_build_id}",
        org_id=state.org_id,
        project_id=project.project_id,
        user_id=f"{_random_string()}@{state.org_id}",
        version=str(uuid4()),
    )


def get_mock_client(state: MockState) -> MagicMock:
    return MagicMock(state=state)


################################################################################
# ENDPOINT MOCKS:
################################################################################


def _signature_parameters_match(subject: Signature, mock: Signature) -> bool:
    """Helper to check that two signatures match."""

    for p in mock.parameters:
        if p not in subject.parameters:
            return False

    for name, param in subject.parameters.items():
        if name not in mock.parameters:
            return False
        mock_param = mock.parameters[name]

        # Special case for clients
        if name == "client":
            if (
                param.annotation is not AuthenticatedClient
                or mock_param.annotation is not MagicMock
            ):
                return False
        elif param.annotation is not mock_param.annotation:
            return False
    return subject.return_annotation is mock.return_annotation


def mocks_endpoint(subject: Callable) -> Callable:
    """Decorator factory to make sure that mocks match the subject signature.

    In practice, this guarantees that the parameters of each function match and their type
    annotations match (except for client which is enforced to be a MagicMock in the mock case and
    AuthenticatedClient in the endpoint case).
    """
    subject_sig = signature(subject)

    def impl(f: Callable) -> Callable:
        sig = signature(f)
        if not _signature_parameters_match(subject_sig, sig):
            raise ValueError("Mock signature mismatch!")
        return f

    return impl


@mocks_endpoint(get_batch.asyncio)
async def get_batch_asyncio(
    project_id: str,
    batch_id: str,
    *,
    client: MagicMock,
) -> Optional[Union[Any, Batch]]:
    batch = client.state.batches[UUID(batch_id)]
    if project_id != batch.project_id:
        raise ValueError("Project id mismatch!")
    return batch


@mocks_endpoint(get_test_suite.asyncio)
async def get_test_suite_asyncio(
    project_id: str,
    test_suite_id: str,
    *,
    client: MagicMock,
) -> Optional[Union[Any, TestSuite]]:
    test_suite = client.state.test_suites[UUID(test_suite_id)]
    if project_id != test_suite.project_id:
        raise ValueError("Project id mismatch!")
    return test_suite


@mocks_endpoint(create_report.asyncio)
async def create_report_asyncio(
    project_id: str,
    *,
    client: MagicMock,
    body: ReportInput,
) -> Optional[Union[Any, Report]]:
    report_id = uuid4()
    # TODO(michael) Enforce some consistency requirements
    report = Report(
        associated_account="",
        branch_id=body.branch_id,
        creation_timestamp=datetime.now(tz=timezone.utc),
        end_timestamp=body.end_timestamp,
        last_updated_timestamp=datetime.now(tz=timezone.utc),
        metrics_build_id=body.metrics_build_id,
        metrics_status=MetricStatus.NO_STATUS_REPORTED,
        name=body.name,
        org_id=client.state.org_id,
        output_location="",
        project_id=project_id,
        report_id=str(report_id),
        respect_revision_boundary=body.respect_revision_boundary,
        start_timestamp=body.start_timestamp,
        status=ReportStatus.SUBMITTED,
        status_history=[],
        test_suite_id=body.test_suite_id,
        test_suite_revision=body.test_suite_revision,
        user_id=f"simbox@{client.state.org_id}",
    )
    client.state.reports[report_id] = report
    return report


def make_mock_state() -> MockState:
    """Helper to create a MockState containing what we need for our tests."""
    state = MockState()

    for _ in range(5):
        p = random_project(state)
        state.projects[UUID(p.project_id)] = p

    for _ in range(5):
        mb = random_metrics_build(state)
        state.metrics_builds[UUID(mb.metrics_build_id)] = mb

    for _ in range(5):
        ts = random_test_suite(state)
        state.test_suites[UUID(ts.test_suite_id)] = ts

    for _ in range(5):
        b = random_batch(state)
        state.batches[UUID(b.batch_id)] = b

    return state
