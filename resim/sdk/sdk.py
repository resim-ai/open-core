from contextlib import contextmanager
from dataclasses import dataclass, field
from enum import Enum
import mimetypes
import os
import tempfile
from typing import List, Optional
from uuid import UUID
from resim.metrics.python.emissions import Emitter
from pathlib import Path
from resim_python_client.models.create_system_input import CreateSystemInput
from resim_python_client.models.job_status import JobStatus
from resim_python_client.client import AuthenticatedClient
from resim.auth.python.device_code_client import DeviceCodeClient
from resim.auth.python.username_password_client import UsernamePasswordClient
from resim_python_client.api.batches import list_batches
from resim_python_client.api.builds import list_builds_for_branches, create_build_for_branch
from resim_python_client.api.projects import (
    list_projects,
    create_branch_for_project,
    list_branches_for_project,
)
from resim_python_client.api.systems import list_systems
from resim_python_client.api.test_suites import list_test_suites
from resim_python_client.api.test_suites import create_test_suite
from resim.metrics.fetch_all_pages import fetch_all_pages
from resim_python_client.models.project import Project
from resim_python_client.models.system import System
from resim_python_client.models.architecture import Architecture
from resim_python_client.models.test_suite import TestSuite
from resim_python_client.models.create_test_suite_input import CreateTestSuiteInput
from resim_python_client.models.create_branch_input import CreateBranchInput
from resim_python_client.models.create_build_for_branch_input import CreateBuildForBranchInput
from resim_python_client.models.branch_type import BranchType
from resim_python_client.models.branch import Branch
from resim_python_client.api.systems import create_system
from resim_python_client.api.experiences import list_experiences
from resim_python_client.models.experience import Experience
from resim_python_client.api.experiences import create_experience
from resim_python_client.models.create_experience_input import CreateExperienceInput
from resim_python_client.api.test_suites import add_experiences_to_test_suite
from resim_python_client.api.batches import create_batch_for_test_suite
from resim_python_client.models.test_suite_batch_input import TestSuiteBatchInput
from resim_python_client.models.select_experiences_input import SelectExperiencesInput
from resim_python_client.api.batches import list_tasks_and_jobs_for_run_counter
from resim_python_client.models.list_tasks_and_jobs_for_run_counter_output import ListTasksAndJobsForRunCounterOutput

@dataclass
class Log:
    filename: str
    path: Path
    size: int
    log_type: "LogType"


class LogType(Enum):
    IMAGE = "image"
    MP4 = "mp4"
    MCAP = "mcap"
    ZIP = "zip"
    OTHER = "other"


def _determine_log_type(path: Path) -> LogType:
    mime, _ = mimetypes.guess_type(str(path))
    suffixes = [s.lower() for s in path.suffixes]

    if any(s == ".mcap" for s in suffixes):
        return LogType.MCAP
    if mime is not None:
        if mime.startswith("image/"):
            return LogType.IMAGE
        if mime == "video/mp4":
            return LogType.MP4
        if mime in ("application/zip", "application/x-zip-compressed"):
            return LogType.ZIP
    # Fallbacks based on extensions if MIME is None or unrecognized
    if any(s == ".mp4" for s in suffixes):
        return LogType.MP4
    if any(s == ".zip" for s in suffixes):
        return LogType.ZIP
    return LogType.OTHER

@dataclass
class Test(Emitter):
    name: str
    emissions_file: Path
    status: JobStatus
    logs: list[Log] = field(default_factory=list)

    def __init__(self, name: str, filename: Path):
        self.name = name
        self.emissions_file = filename
        self.logs = []
        super().__init__(output_path=filename)

    def add_file(self, filename: Path) -> Log:
        path = filename if isinstance(filename, Path) else Path(filename)
        if not path.exists() or not path.is_file():
            raise ValueError(f"File not found or not a regular file: {path}")

        size = path.stat().st_size
        log_type = _determine_log_type(path)
        log = Log(filename=path.name, path=path, size=size, log_type=log_type)
        self.logs.append(log)
        return log


@dataclass
class Batch:
    name: str
    project: str
    system: str
    branch: Optional[str] = None
    version: Optional[str] = None
    tests: list[Test] = field(default_factory=list)
    experience_name_to_id: dict[str, str] = field(default_factory=dict)

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
        self.experience_name_to_id = {}

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
                    build_shared_memory_mb=64,
                    architecture=Architecture.AMD64,
                    metrics_build_vcpus=1,
                    metrics_build_memory_mib=1024,
                    metrics_build_gpus=0,
                    metrics_build_shared_memory_mb=64,
                ),
            )
            if new_system_resp is None:
                raise RuntimeError(f"Failed to create system {system}")
            system_id = new_system_resp.system_id
        # Ensure branch exists (upsert behavior)
        branch_id = upsert_branch_id(auth_client, project_id, branch) if branch else None
        build_id = upsert_build_id(auth_client, project_id, branch_id, version, system, system_id)
        print(f"Created build {build_id} for branch {branch_id}")
        
        suite_id = None
        experience_name_to_id = {}
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
            experience_map = upsert_experiences_map(
                auth_client,
                project_id,
                system_id,
                [t.name for t in batch_obj.tests],
            )
            batch_obj.experience_name_to_id = experience_map
            new_suite_resp = create_test_suite.sync(
                project_id,
                client=auth_client,
                body=CreateTestSuiteInput(
                    name=test_suite,
                    description="",
                    system_id=system_id,
                    experiences=list(experience_map.values()),
                ),
            )
            if new_suite_resp is None:
                raise RuntimeError(f"Failed to create test suite {test_suite}")
            
            suite_id = new_suite_resp.test_suite_id

            # create a new batch for this test suite, with type "LITE"
            print(f"Created test suite {suite_id} with build {build_id}")
        else:
            experience_map = upsert_experiences_map(
                auth_client,
                project_id,
                system_id,
                [t.name for t in batch_obj.tests],
            )
            batch_obj.experience_name_to_id = experience_map
            update_suite_resp = add_experiences_to_test_suite.sync(
                project_id,
                client=auth_client,
                test_suite_id=suite_id,
                body=SelectExperiencesInput(
                    experiences=list(experience_map.values()),
                ),
            )
            if update_suite_resp is None:
                raise RuntimeError(
                    f"Failed to add experiences to test suite {test_suite}"
                )
            suite_id = update_suite_resp.test_suite_id
        

        if suite_id is None:
            raise RuntimeError(f"Failed to create test suite {test_suite}") 
        batch = create_the_batch(build_id, batch, suite_id, project_id, auth_client)
        # list the jobs and tasks for this batch:
        tasks_and_jobs = list_tasks_and_jobs_for_run_counter.sync(
            project_id,
            batch_id=batch.batch_id,
            run_counter=0,
            client=auth_client,
        )
        if tasks_and_jobs is None:
            raise RuntimeError(f"Failed to list tasks and jobs for batch {batch.batch_id}")
        for jobAndTask in tasks_and_jobs.tasks:
            job = jobAndTask.job
            task = jobAndTask.task
            print(f"Job {job.job_id} and task {task.task_id}")
            # upload the emissions file for this job
        print(f"Tasks and jobs for batch {batch.batch_id}: {tasks_and_jobs}")

def is_valid_uuid(uuid: str) -> bool:
    try:
        UUID(uuid)
        return True
    except ValueError:
        return False

def create_the_batch(build_id: str, batch_name: str, suite_id: str, project_id: str, auth_client: AuthenticatedClient) -> any:
    pool_labels= ["resim:k8s:metrics2:lite"]
    body = TestSuiteBatchInput(
        build_id=build_id,
        pool_labels=pool_labels,
        batch_name=batch_name,
    )
    created = create_batch_for_test_suite.sync(
        project_id,
        client=auth_client,
        test_suite_id=suite_id,
        body=body,
    )
    if created is None:
        raise RuntimeError(f"Failed to create batch for test suite {test_suite}")
    return created

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
                location=f"local-experience-{experience_name}",
            ),
        )
        if create_experience_resp is None:
            raise RuntimeError(f"Failed to create experience {experience_name}")
        experience = create_experience_resp
    return experience.experience_id


def upsert_experiences_map(
    client: AuthenticatedClient,
    project_id: str,
    system_id: str,
    experience_names: list[str],
) -> dict[str, str]:
    mapping: dict[str, str] = {}
    # de-duplicate while preserving order
    seen: set[str] = set()
    for name in experience_names:
        if name in seen:
            continue
        seen.add(name)
        exp_id = upsert_experience(client, project_id, system_id, name)
        if exp_id is None:
            raise RuntimeError(f"Failed to upsert experience {name}")
        mapping[name] = exp_id
    return mapping


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

def upsert_branch_id(
    client: AuthenticatedClient, project_id: str, branch_name: str, branch_type: BranchType = BranchType.MAIN
) -> str:
    branches: List[Branch] = [
        b
        for page in fetch_all_pages(
            list_branches_for_project.sync, project_id, client=client, name=branch_name
        )
        for b in getattr(page, "branches", [])
    ]
    branch = next((b for b in branches if b.name == branch_name), None)
    if branch is not None:
        return branch.branch_id

    created = create_branch_for_project.sync(
        project_id,
        client=client,
        body=CreateBranchInput(
            name=branch_name,
            branch_type=branch_type,
        ),
    )
    if created is None:
        raise RuntimeError(f"Failed to create branch {branch_name}")
    return created.branch_id

def upsert_build_id(
    client: AuthenticatedClient, project_id: str, branch_id: str, build_version: str, system_name: str, system_id: str
) -> str:
    builds: List[Build] = [
        b
        for page in fetch_all_pages(
            list_builds_for_branches.sync, project_id, branch_id, client=client
        )
        for b in getattr(page, "builds", [])
    ]
    build = next((b for b in builds if b.version == build_version), None)
    if build is not None:
        return build.build_id
    created = create_build_for_branch.sync(
        project_id,
        client=client,
        branch_id=branch_id,
        body=CreateBuildForBranchInput(
            version=build_version,
            system_id=system_id,
            name=system_name,
            description="An automated build created by the ReSim SDK",
            image_uri="public.ecr.aws/docker/library/hello-world:latest"
        ),
    )
    if created is None:
        raise RuntimeError(f"Failed to create build {build_version}")
    return created.build_id
    

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
    api_url = "https://dev-env-pr-2455.api.dev.resim.io/v1/"
    auth_url = "https://resim-dev.us.auth0.com"
    client_id = "Rg1F0ZOCBmVYje4UVrS3BKIh4T2nCW9y"

    if os.getenv("RESIM_USERNAME") and os.getenv("RESIM_PASSWORD"):
        return UsernamePasswordClient(
            username=os.getenv("RESIM_USERNAME"),
            password=os.getenv("RESIM_PASSWORD"),
        )
    else:
        auth_client = DeviceCodeClient(domain=auth_url, client_id=client_id)
        token = auth_client.get_jwt()["access_token"]
        client = AuthenticatedClient(base_url=api_url, token=token)
        return client
