from contextlib import contextmanager
from dataclasses import dataclass, field
import os
import tempfile
from typing import List, Optional
from uuid import UUID
from resim.metrics.python.emissions import Emitter
from pathlib import Path
from resim_python_client.models.create_system_input import CreateSystemInput
from resim_python_client.models.job_status import JobStatus
from resim_python_client import AuthenticatedClient
from resim.auth.python.device_code_client import DeviceCodeClient
from resim.auth.python.username_password_client import UsernamePasswordClient
from resim_python_client.api.batches import list_batches
from resim_python_client.api.projects import list_projects
from resim_python_client.api.systems import list_systems
from resim_python_client.api.test_suites import list_test_suites
from resim_python_client.api.test_suites import create_test_suite
from resim.metrics.fetch_all_pages import fetch_all_pages
from resim_python_client.models.project import Project
from resim_python_client.models.system import System
from resim_python_client.models.architecture import Architecture
from resim_python_client.models.test_suite import TestSuite
from resim_python_client.models.create_test_suite_input import CreateTestSuiteInput
from resim_python_client.api.systems import create_system
from resim_python_client.api.experiences import list_experiences
from resim_python_client.models.experience import Experience
from resim_python_client.api.experiences import create_experience
from resim_python_client.models.create_experience_input import CreateExperienceInput
from resim_python_client.api.test_suites import add_experiences_to_test_suite
from resim_python_client.models.select_experiences_input import SelectExperiencesInput


@dataclass
class Test(Emitter):
    name: str
    emissions_file: Path
    status: JobStatus

    def __init__(self, name: str, filename: Path):
        self.name = name
        self.emissions_file = filename
        super().__init__(output_path=filename)

    def add_file(self, filename: Path):
        raise NotImplementedError("Not implemented")


@dataclass
class Batch:
    name: str
    project: str
    system: str
    branch: Optional[str] = None
    version: Optional[str] = None
    tests: list[Test] = field(default_factory=list)

    def __init__(
        self,
        *,
        name: str,
        project: str,
        system: str,
        branch: Optional[str] = None,
        version: Optional[str] = None,
    ):
        self.name = name
        self.project = project
        self.system = system
        self.branch = branch
        self.version = version
        self.tests = []

    @contextmanager
    def run_test(self, name: str):
        # TODO(mattc): ensure we delete the file after it's uploaded
        with tempfile.NamedTemporaryFile(
            suffix=".resim.jsonl", delete=False
        ) as temp_file:
            emissions_file = Path(temp_file.name)
            test = Test(name, emissions_file)
            self.tests.append(test)
            yield test


@contextmanager
def init(
    batch: str,
    project: str,
    system: str,
    test_suite: Optional[str] = None,
    branch: Optional[str] = None,
    version: Optional[str] = None,
):
    batch_obj = Batch(
        name=batch, project=project, system=system, branch=branch, version=version
    )
    auth_client = get_auth_client()
    try:
        yield batch_obj
    finally:
        # check if project looks like a UUID or a name
        project_id = (
            project if is_valid_uuid(project) else get_project_id(auth_client, project)
        )
        system_id = (
            system
            if is_valid_uuid(system)
            else get_system_id(auth_client, project_id, system)
        )
        if system_id is None:
            # create a new system
            new_system_resp = create_system.sync(
                project_id,
                client=auth_client,
                body=CreateSystemInput(
                    name=system,
                    description="",
                    build_vcpus=1,
                    build_memory_mib=1024,
                    build_gpus=0,
                    build_shared_memory_mb=0,
                    architecture=Architecture.AMD64,
                    metrics_build_vcpus=1,
                    metrics_build_memory_mib=1024,
                    metrics_build_gpus=0,
                    metrics_build_shared_memory_mb=0,
                ),
            )
            if new_system_resp is None:
                raise RuntimeError(f"Failed to create system {system}")
            system_id = new_system_resp.system_id
        if test_suite is None:
            test_suite = batch
        suite_id = (
            test_suite
            if is_valid_uuid(test_suite)
            else get_suite_id(auth_client, project_id, test_suite)
        )
        if suite_id is None:
            # create a new test suite
            # TODO(mattc): sync metrics set and add to suite
            new_suite_resp = create_test_suite.sync(
                project_id,
                client=auth_client,
                body=CreateTestSuiteInput(
                    name=test_suite,
                    description="",
                    system_id=system_id,
                    experiences=[
                        upsert_experience(auth_client, project_id, system_id, test.name)
                        for test in batch_obj.tests
                    ],
                ),
            )
            if new_suite_resp is None:
                raise RuntimeError(f"Failed to create test suite {test_suite}")
            suite_id = new_suite_resp.test_suite_id
        else:
            update_suite_resp = add_experiences_to_test_suite.sync(
                project_id,
                client=auth_client,
                test_suite_id=suite_id,
                body=SelectExperiencesInput(
                    experiences=[
                        upsert_experience(auth_client, project_id, system_id, test.name)
                        for test in batch_obj.tests
                    ],
                ),
            )
            if update_suite_resp is None:
                raise RuntimeError(
                    f"Failed to add experiences to test suite {test_suite}"
                )
            suite_id = update_suite_resp.test_suite_id
        # check if the batch exists
        batches_resp = fetch_all_pages(
            list_batches.sync, project_id, text=batch, client=auth_client
        )
        batches = [b for page in batches_resp for b in getattr(page, "batches", [])]

        # If it does, update the test suite with the new tests, call a rerun then upload the results
        # if it doesn't, create a new test suite, run it and then upload the results
        pass


def is_valid_uuid(uuid: str) -> bool:
    try:
        UUID(uuid)
        return True
    except ValueError:
        return False


# TODO: add an endpoint to the customer api to do this :-(
def get_project_id(client: AuthenticatedClient, project_name: str) -> str:
    projects: List[Project] = [
        p
        for page in fetch_all_pages(list_projects.sync, client=client)
        for p in getattr(page, "projects", [])
    ]
    project = next((p for p in projects if p.name == project_name), None)
    if project is None:
        raise ValueError(f"Project {project_name} not found")
    return project.project_id


def upsert_experience(
    client: AuthenticatedClient, project_id: str, system_id: str, experience_name: str
) -> Optional[str]:
    experiences: List[Experience] = [
        e
        for page in fetch_all_pages(
            list_experiences.sync, project_id, client=client, text=experience_name
        )
        for e in getattr(page, "experiences", [])
    ]
    experience = next((e for e in experiences if e.name == experience_name), None)
    if experience is None:
        create_experience_resp = create_experience.sync(
            project_id,
            client=client,
            body=CreateExperienceInput(
                name=experience_name,
                description="",
                system_i_ds=[system_id],
            ),
        )
        if create_experience_resp is None:
            raise RuntimeError(f"Failed to create experience {experience_name}")
        experience = create_experience_resp
    return experience.experience_id


def get_system_id(
    client: AuthenticatedClient, project_id: str, system_name: str
) -> Optional[str]:
    systems: List[System] = [
        s
        for page in fetch_all_pages(
            list_systems.sync, project_id, client=client, name=system_name
        )
        for s in getattr(page, "systems", [])
    ]
    system = next((s for s in systems if s.name == system_name), None)
    if system is None:
        return None
    return system.system_id


def get_suite_id(
    client: AuthenticatedClient, project_id: str, test_suite_name: str
) -> Optional[str]:
    suites: List[TestSuite] = [
        s
        for page in fetch_all_pages(
            list_test_suites.sync, project_id, client=client, name=test_suite_name
        )
        for s in getattr(page, "test_suites", [])
    ]
    suite = next((s for s in suites if s.name == test_suite_name), None)
    if suite is None:
        return None
    return suite.test_suite_id


def get_auth_client() -> AuthenticatedClient:
    if os.getenv("RESIM_USERNAME") and os.getenv("RESIM_PASSWORD"):
        return UsernamePasswordClient(
            username=os.getenv("RESIM_USERNAME"),
            password=os.getenv("RESIM_PASSWORD"),
        )
    else:
        device_code_client = DeviceCodeClient()
        return AuthenticatedClient(
            base_url="https://api.resim.ai/v1/",
            token=device_code_client.get_jwt()["access_token"],
        )
