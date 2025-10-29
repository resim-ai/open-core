from contextlib import contextmanager
from dataclasses import dataclass, field
import os
import tempfile
from typing import List, Optional
from uuid import UUID
from resim.metrics.python.emissions import Emitter
from pathlib import Path
from resim_python_client.models.job_status import JobStatus
from resim_python_client import AuthenticatedClient
from resim.auth.python.device_code_client import DeviceCodeClient
from resim.auth.python.username_password_client import UsernamePasswordClient
from resim_python_client.api.batches import list_batches
from resim_python_client.api.projects import list_projects
from resim.metrics.fetch_all_pages import fetch_all_pages
from resim_python_client.models.project import Project


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
    branch: Optional[str] = None,
    version: Optional[str] = None,
):
    auth_client = get_auth_client()
    try:
        yield Batch(
            name=batch, project=project, system=system, branch=branch, version=version
        )
    finally:
        # check if project looks like a UUID or a name
        project_id = (
            project if is_valid_uuid(project) else get_project_id(auth_client, project)
        )
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
