from typing import List, Optional
from uuid import UUID

from resim.metrics.fetch_all_pages import fetch_all_pages
from resim_python_client.client import AuthenticatedClient

# Projects
from resim_python_client.api.projects import (
    list_projects,
    create_branch_for_project,
    list_branches_for_project,
)
from resim_python_client.models.project import Project
from resim_python_client.models.create_branch_input import CreateBranchInput
from resim_python_client.models.branch_type import BranchType
from resim_python_client.models.branch import Branch

# Systems
from resim_python_client.api.systems import list_systems, create_system
from resim_python_client.models.system import System
from resim_python_client.models.create_system_input import CreateSystemInput
from resim_python_client.models.architecture import Architecture

# Test suites / experiences
from resim_python_client.api.experiences import list_experiences, create_experience
from resim_python_client.models.experience import Experience
from resim_python_client.models.create_experience_input import CreateExperienceInput
from resim_python_client.api.test_suites import (
    list_test_suites,
    create_test_suite,
    add_experiences_to_test_suite,
)
from resim_python_client.models.test_suite import TestSuite
from resim_python_client.models.create_test_suite_input import CreateTestSuiteInput
from resim_python_client.models.select_experiences_input import SelectExperiencesInput

# Builds
from resim_python_client.api.builds import (
    list_builds_for_branches,
    create_build_for_branch,
)
from resim_python_client.models.create_build_for_branch_input import (
    CreateBuildForBranchInput,
)
from resim_python_client.models.build import Build


def is_valid_uuid(uuid: str) -> bool:
    try:
        UUID(uuid)
        return True
    except ValueError:
        return False


def get_project_id(client: AuthenticatedClient, project: str) -> str:
    if is_valid_uuid(project):
        return project
    projects: List[Project] = [
        p
        for page in fetch_all_pages(list_projects.sync, client=client)
        for p in getattr(page, "projects", [])
    ]
    match = next((p for p in projects if p.name == project), None)
    if match is None:
        raise ValueError(f"Project {project} not found")
    return match.project_id


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


def upsert_system_id(
    client: AuthenticatedClient, project_id: str, system: str
) -> str:
    if is_valid_uuid(system):
        return system
    existing_id = get_system_id(client, project_id, system)
    if existing_id is not None:
        return existing_id
    created = create_system.sync(
        project_id,
        client=client,
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
    if created is None:
        raise RuntimeError(f"Failed to create system {system}")
    return created.system_id


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
            image_uri="public.ecr.aws/docker/library/hello-world:latest",
        ),
    )
    if created is None:
        raise RuntimeError(f"Failed to create build {build_version}")
    return created.build_id


def create_or_revise_test_suite(
    client: AuthenticatedClient,
    project_id: str,
    system_id: str,
    test_suite_name: str,
    experience_names: list[str],
) -> tuple[str, dict[str, str]]:
    """Create a test suite if missing or add experiences to an existing one.

    Returns (suite_id, experience_name_to_id_map).
    """
    suite_id = get_suite_id(client, project_id, test_suite_name)
    experience_map = upsert_experiences_map(
        client,
        project_id,
        system_id,
        experience_names,
    )

    if suite_id is None:
        new_suite_resp = create_test_suite.sync(
            project_id,
            client=client,
            body=CreateTestSuiteInput(
                name=test_suite_name,
                description="",
                system_id=system_id,
                experiences=list(experience_map.values()),
            ),
        )
        if new_suite_resp is None:
            raise RuntimeError(f"Failed to create test suite {test_suite_name}")
        suite_id = new_suite_resp.test_suite_id
    else:
        # Fetch existing suite to get its current experiences
        suites_page = list_test_suites.sync(project_id=project_id, client=client, name=test_suite_name)
        existing_ids: set[str] = set()
        if suites_page is not None:
            for s in getattr(suites_page, "test_suites", []) or []:
                if s.name == test_suite_name:
                    existing_ids = set(getattr(s, "experiences", []) or [])
                    break
        desired_ids: list[str] = list(experience_map.values())
        to_add = [eid for eid in desired_ids if eid not in existing_ids]
        if to_add:
            update_suite_resp = add_experiences_to_test_suite.sync(
                project_id,
                client=client,
                test_suite_id=suite_id,
                body=SelectExperiencesInput(
                    experiences=to_add,
                ),
            )
            if update_suite_resp is None:
                raise RuntimeError(
                    f"Failed to add experiences to test suite {test_suite_name}"
                )
            suite_id = update_suite_resp.test_suite_id

    return suite_id, experience_map
