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
from resim_python_client.models.job_status import JobStatus
from resim_python_client.client import AuthenticatedClient
from resim.auth.python.device_code_client import DeviceCodeClient
from resim.auth.python.username_password_client import UsernamePasswordClient
from resim_python_client.api.batches import create_batch_for_test_suite
from resim_python_client.models.test_suite_batch_input import TestSuiteBatchInput
from resim_python_client.api.batches import list_tasks_and_jobs_for_run_counter

from resim.sdk.sdk_helpers import (
    is_valid_uuid,
    get_project_id,
    upsert_system_id,
    upsert_experiences_map,
    upsert_branch_id,
    get_suite_id,
    upsert_build_id,
    create_or_revise_test_suite,
)

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
        project_id = get_project_id(auth_client, project)
        system_id = upsert_system_id(auth_client, project_id, system)
        # Ensure branch exists (upsert behavior)
        branch_id = upsert_branch_id(auth_client, project_id, branch) if branch else None
        build_id = upsert_build_id(auth_client, project_id, branch_id, version, system, system_id)
        print(f"Created build {build_id} for branch {branch_id}")
        
        suite_id, experience_map = create_or_revise_test_suite(
            auth_client,
            project_id,
            system_id,
            test_suite or batch,
            [t.name for t in batch_obj.tests],
        )
        batch_obj.experience_name_to_id = experience_map
        print(f"Prepared test suite {suite_id} with build {build_id}")
        

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
