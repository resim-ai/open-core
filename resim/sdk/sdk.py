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
from resim_python_client.models.log_type import LogType

from resim.sdk.sdk_helpers import (
    is_valid_uuid,
    get_project_id,
    upsert_system_id,
    upsert_experiences_map,
    upsert_branch_id,
    get_suite_id,
    upsert_build_id,
    create_or_revise_test_suite,
    upload_logs_and_update_task_status_for_batch_jobs,
    create_or_rerun_the_batch,
)

@dataclass
class Log:
    filename: str
    path: Path
    size: int
    log_type: "LogType"



def _determine_log_type(path: Path) -> LogType:
    mime, _ = mimetypes.guess_type(str(path))
    suffixes = [s.lower() for s in path.suffixes]

    if any(s == ".mcap" for s in suffixes):
        return LogType.MCAP_LOG
    if mime is not None:
        if mime == "video/mp4":
            return LogType.MP4_LOG
        if mime in ("application/zip", "application/x-zip-compressed"):
            return LogType.ZIP_LOG
        
    # Fallbacks based on extensions if MIME is None or unrecognized
    if any(s == ".mp4" for s in suffixes):
        return LogType.MP4_LOG
    if any(s == ".zip" for s in suffixes):
        return LogType.ZIP_LOG
    if any(s == ".jsonl" for s in suffixes):
        return LogType.EMISSIONS_LOG
    return LogType.OTHER_LOG

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
        self.add_file(filename)
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
    metrics_set_name: Optional[str] = None
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
        metrics_set_name: Optional[str] = None,
        branch: Optional[str] = None,
        version: Optional[str] = None,
    ):
        self.name = name
        self.project = project
        self.system = system
        self.metrics_set_name = metrics_set_name
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
    metrics_set_name: Optional[str] = None,
    test_suite: Optional[str] = None,
    branch: Optional[str] = None,
    version: Optional[str] = None,
):
    batch_obj = Batch(
        name=batch, project=project, system=system, branch=branch, version=version, metrics_set_name=metrics_set_name
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
            metrics_set_name=metrics_set_name,
        )
        batch_obj.experience_name_to_id = experience_map
        print(f"Prepared test suite {suite_id} with build {build_id}")
        

        if suite_id is None:
            raise RuntimeError(f"Failed to create test suite {test_suite}") 
        
        batch_id,run_counter = create_or_rerun_the_batch(build_id, batch, suite_id, project_id, auth_client)
        print(f"Created or rerun batch {batch_id} with run counter {run_counter}")
        tasks_and_jobs = upload_logs_and_update_task_status_for_batch_jobs(
            auth_client,
            project_id,
            batch_obj,
            batch_id,
            run_counter,
        )
 


def get_auth_client() -> AuthenticatedClient:
    api_url = "https://dev-env-pr-2467.api.dev.resim.io/v1/"
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
