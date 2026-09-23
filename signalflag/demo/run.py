# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Run the SignalFlag SDK demo: two comparable batches, and links to the results."""

import json
import shutil
import sys
from dataclasses import dataclass
from importlib import resources
from pathlib import Path
from typing import Any, Iterator, Optional, Union

from httpx import URL

from signalflag.demo import bundle, links
from signalflag.demo.bundle import Bundle, DemoDataError
from signalflag.sdk.auth import env
from signalflag.sdk.auth.const import (
    DEFAULT_BASE_URL,
    default_cache_location,
    DEFAULT_DOMAIN,
)
from signalflag.sdk.auth.device_code_client import (
    DEVICE_CODE_CLIENT_ID,
    DeviceCodeClient,
)
from signalflag.sdk.batch import Batch
from signalflag.sdk.bff_client.dashboards import find_dashboard_id
from signalflag.sdk.client import AuthenticatedClient
from signalflag.sdk.client.api.experiences import create_experience, list_experiences
from signalflag.sdk.client.api.experience_tags import (
    add_experience_tag_to_experience,
    create_experience_tag,
    list_experience_tags,
)
from signalflag.sdk.client.api.projects import create_project, list_projects
from signalflag.sdk.client.api.systems import create_system, list_systems
from signalflag.sdk.client.api.test_suites import (
    create_test_suite,
    list_test_suites,
    revise_test_suite,
)
from signalflag.sdk.client.models.architecture import Architecture
from signalflag.sdk.client.models.create_experience_input import (
    CreateExperienceInput,
)
from signalflag.sdk.client.models.create_experience_tag_input import (
    CreateExperienceTagInput,
)
from signalflag.sdk.client.models.create_project_input import CreateProjectInput
from signalflag.sdk.client.models.create_system_input import CreateSystemInput
from signalflag.sdk.client.models.create_test_suite_input import CreateTestSuiteInput
from signalflag.sdk.client.models.revise_test_suite_input import ReviseTestSuiteInput
from signalflag.sdk.client.models.test_suite import TestSuite
from signalflag.sdk.test import LogType, Test

__all__ = ["DEMOS", "Demo", "DemoResult", "config_path", "run", "templates_path"]

PROJECT_DESCRIPTION = "Created by the SignalFlag Python SDK demo (signalflag.demo.run)."


@dataclass(frozen=True)
class Demo:
    """One runnable demo: its data, its metrics config, and where it lands.

    Everything that differs between demos lives here, so adding another is a
    registry entry plus a config file rather than a change to ``run``.
    """

    #: Value of ``--demo``.
    key: str
    #: One line, shown in ``--help``.
    summary: str
    project_name: str
    branch: str
    bundle: bundle.BundleSource
    #: Metrics config shipped in ``signalflag/demo/data``.
    config_file: str
    metrics_set: str
    #: Name of the dashboard this demo's config defines. None for a demo with
    #: nothing to trend by build version - it skips dashboard resolution and
    #: reporting entirely rather than pointing at an empty one.
    dashboard_name: Optional[str] = None
    #: Manifest keys this demo's bundle defines, one batch each, all on the
    #: same branch. Two sides get a compare link between them; any other
    #: count does not, since "compare" only means something for a pair.
    sides: tuple[str, ...] = ("a", "b")
    #: System each batch is attached to. Created if it does not exist yet.
    #: None skips system resolution entirely.
    system: Optional[str] = None
    #: Experience tag applied to every experience this demo's batches create.
    #: None skips tagging entirely.
    experience_tag: Optional[str] = None
    #: Test suite every batch runs against, holding one experience per job
    #: across all sides. Requires ``system``. None runs each batch ad hoc.
    test_suite: Optional[str] = None


DEMOS: dict[str, Demo] = {
    "navigation": Demo(
        key="navigation",
        summary=(
            "A hospital navigation suite: dense telemetry across 34 scenarios, "
            "covering every chart type ReSim ships."
        ),
        project_name="SignalFlag SDK Demo",
        branch="sdk-demo",
        bundle=bundle.BundleSource(
            url=(
                "https://resim-public-assets.s3.us-east-1.amazonaws.com"
                "/sdk-demo/resim-sdk-demo-data-v1.tar.gz"
            ),
            sha256="7bc5d26c14aaad8d849d131276af7c7287d163aeb3a9d2892f0ffba21889935d",
            cache_key="navigation-v1",
        ),
        config_file="config.resim.yml",
        metrics_set="Demo Metrics",
        dashboard_name="SDK Demo Trends",
    ),
    "mujoco": Demo(
        key="mujoco",
        summary=(
            "An ALOHA bimanual manipulation policy in MuJoCo: one test per "
            "cube placement, compared across two policy builds."
        ),
        project_name="SignalFlag SDK Demo (MuJoCo)",
        branch="sdk-demo-mujoco",
        bundle=bundle.BundleSource(
            url=(
                "https://resim-public-assets.s3.us-east-1.amazonaws.com"
                "/sdk-demo/resim-sdk-demo-mujoco-v4.tar.gz"
            ),
            sha256="bde0c251f3d0df421c004aae7969617aa71596678228787026c1e96c87d9da2f",
            cache_key="mujoco-v4",
        ),
        config_file="mujoco.resim.yml",
        metrics_set="ALOHA Metrics",
        dashboard_name="ALOHA Policy Trends",
    ),
    "session": Demo(
        key="session",
        summary=(
            "Four real field sessions from a legged robot: GNSS, IMU and gait "
            "telemetry, one batch per session on a shared branch."
        ),
        project_name="SignalFlag SDK Demo (Logs to Insights)",
        branch="sdk-demo-logs-to-insights",
        bundle=bundle.BundleSource(
            url=(
                "https://resim-public-assets.s3.us-east-1.amazonaws.com"
                "/sdk-demo/resim-sdk-demo-session-v1.tar.gz"
            ),
            sha256="ffdae98bfd9e4ef85f65691d82dd57d3ae10f3229fd96b9ea2dab7ccf4c7ad60",
            cache_key="session-v1",
        ),
        config_file="session.resim.yml",
        metrics_set="Session Metrics",
        sides=("2024-11-04", "2024-11-14", "2024-11-15", "2024-11-18"),
        system="Session Evaluations",
        experience_tag="resim-session",
        test_suite="Field Sessions",
    ),
}


def get_demo(key: str) -> Demo:
    """Look a demo up by ``--demo`` value.

    Raises:
        DemoDataError: If no demo goes by that name.
    """
    try:
        return DEMOS[key]
    except KeyError:
        raise DemoDataError(
            f"unknown demo {key!r}. Available: {', '.join(sorted(DEMOS))}"
        ) from None


# Environment overrides for pointing the demo at a non-production deployment.
# Undocumented on the command line on purpose: customers should never need
# them, but we do, to smoke-test against staging. Each is also honoured under
# its legacy RESIM_* spelling; see signalflag.sdk.auth.env.
ENV_API_URL = env.API_URL
ENV_AUTH_DOMAIN = env.AUTH_DOMAIN
ENV_CLIENT_ID = env.CLIENT_ID


@dataclass(frozen=True)
class DemoResult:
    """Where the demo put its results."""

    project_id: str
    batch_ids: dict[str, str]
    dashboard_id: Optional[str]
    urls: dict[str, str]
    test_suite_id: Optional[str] = None


def run(
    project_name: Optional[str] = None,
    *,
    demo: str,
    client: Optional[AuthenticatedClient] = None,
    branch: Optional[str] = None,
    data_dir: Optional[Union[str, Path]] = None,
    quiet: bool = False,
) -> DemoResult:
    """Populate a ReSim project with the demo's batches and print the links.

    Creates the project if it does not exist, syncs the demo's metrics config
    (which also creates the branch and the trends dashboard), then replays real
    test data into one light batch per side on that branch. Two sides differ
    by build version and get an A/B comparison link; any other count trends on
    the dashboard instead.

    Args:
        project_name: Project to run in. Created if it does not exist.
            Defaults to the chosen demo's own project name.
        demo: Which demo to run. See :data:`DEMOS`.
        client: An authenticated ReSim API client. Defaults to interactive
            device code authentication against production.
        branch: Branch to create the batches on. Every side shares it.
            Defaults to the chosen demo's own branch.
        data_dir: An already-extracted demo data bundle to replay instead of
            downloading one. Mainly useful for development.
        quiet: Suppress progress and result output.

    Returns:
        A :class:`DemoResult` with the IDs and URLs, so callers can use them
        programmatically rather than scraping stdout.

    Raises:
        DemoDataError: If the demo's replay data cannot be fetched.
    """

    def say(message: str = "") -> None:
        if not quiet:
            print(message, flush=True)

    chosen = get_demo(demo)
    project_name = chosen.project_name if project_name is None else project_name
    branch = chosen.branch if branch is None else branch

    # Fetch the data before authenticating: there is no point sending someone
    # through a browser login only to fail on a download afterwards.
    data = bundle.ensure(chosen.bundle, data_dir, None if quiet else say)

    if client is None:
        client = default_client()

    project_id = resolve_project(client, project_name, say)

    system_id = (
        resolve_system(client, project_id, chosen.system, say)
        if chosen.system
        else None
    )
    tag_id = (
        resolve_experience_tag(client, project_id, chosen.experience_tag, say)
        if chosen.experience_tag
        else None
    )

    suite: Optional[TestSuite] = None
    if chosen.test_suite:
        if system_id is None:
            raise ValueError(f"demo {chosen.key!r} has a test suite but no system")
        # The suite has to exist before the first batch attaches to it, so its
        # experiences are created up front; each job then matches its own
        # experience by name instead of creating a new one.
        experience_ids = [
            resolve_experience(client, project_id, str(job["experience_name"]), say)
            for side in chosen.sides
            for job in data.jobs(side)
        ]
        suite = resolve_test_suite(
            client,
            project_id,
            chosen.test_suite,
            system_id,
            experience_ids,
            chosen.metrics_set,
            say,
        )

    config_resource, templates_resource = _package_data(chosen)

    batch_ids: dict[str, str] = {}
    job_ids: dict[str, str] = {}
    dashboard_id: Optional[str] = None
    with (
        resources.as_file(config_resource) as config,
        resources.as_file(templates_resource) as templates,
    ):
        for side in chosen.sides:
            details = data.batch(side)
            jobs = data.jobs(side)
            say(
                f"Running batch {side.upper()}: {details.get('name')} "
                f"({len(jobs)} tests)"
            )
            with Batch(
                client=client,
                project_id=project_id,
                branch=branch,
                name=str(details.get("name") or f"SDK Demo {side.upper()}"),
                version=str(details.get("version") or ""),
                metrics_set_name=chosen.metrics_set,
                metrics_config_path=str(config),
                templates_path=str(templates),
                system=chosen.system,
                test_suite=chosen.test_suite,
            ) as batch:
                batch_ids[side] = batch.id
                if dashboard_id is None and chosen.dashboard_name:
                    dashboard_id = find_dashboard_id(
                        client,
                        project_id,
                        batch.branch_id,
                        chosen.dashboard_name,
                    )
                tests: list[Test] = []
                try:
                    for job in jobs:
                        test = replay_job(client, batch, data, job)
                        tests.append(test)
                        if tag_id:
                            tag_experience(
                                client, project_id, tag_id, test.experience_id
                            )
                        say(f"  {job.get('experience_name')}")
                finally:
                    close_errors = []
                    for test in tests:
                        try:
                            test.close()
                        except Exception as e:  # noqa: BLE001 - collected, then re-raised
                            close_errors.append(e)
                    # Only surface a close failure if nothing else is already
                    # propagating: a bare raise here would otherwise displace
                    # the original error the caller needs to see.
                    if close_errors and sys.exc_info()[0] is None:
                        raise close_errors[0]
                # A single-job batch is one session, not an A/B suite; link
                # straight to it instead of to the (otherwise identical) batch.
                if len(tests) == 1:
                    job_ids[side] = tests[0].job_id

    urls = _urls(
        client,
        project_id,
        batch_ids,
        dashboard_id,
        job_ids,
        sessions=bool(chosen.experience_tag),
        has_dashboard=bool(chosen.dashboard_name),
        suite=suite,
    )
    if not quiet:
        _report(
            urls,
            chosen.sides,
            {side: str(data.batch(side).get("version") or "") for side in chosen.sides},
        )

    return DemoResult(
        project_id=project_id,
        batch_ids=batch_ids,
        dashboard_id=dashboard_id,
        urls=urls,
        test_suite_id=suite.test_suite_id if suite else None,
    )


def default_client() -> AuthenticatedClient:
    """Authenticate interactively, honouring the deployment env overrides."""
    base_url = env.getenv(ENV_API_URL)
    domain = env.getenv(ENV_AUTH_DOMAIN)
    client_id = env.getenv(ENV_CLIENT_ID)
    if not (base_url or domain or client_id):
        return DeviceCodeClient()

    # Tokens from different deployments are not interchangeable, so give a
    # non-production login its own cache rather than overwriting the one the
    # user's production sessions rely on.
    cache = default_cache_location()
    if base_url:
        cache = cache.with_name(f"token-{URL(base_url).host}.json")
    return DeviceCodeClient(
        base_url=base_url or DEFAULT_BASE_URL,
        domain=domain or DEFAULT_DOMAIN,
        client_id=client_id or DEVICE_CODE_CLIENT_ID,
        cache_location=cache,
    )


def resolve_project(
    client: AuthenticatedClient, name: str, say: Any = lambda _: None
) -> str:
    """Return the ID of the project called ``name``, creating it if needed."""
    page_token: Optional[str] = None
    while True:
        kwargs: dict[str, Any] = {"client": client}
        if page_token:
            kwargs["page_token"] = page_token
        response = list_projects.sync(**kwargs)
        assert response is not None, "failed to fetch projects"
        for project in response.projects or []:
            if project.name == name:
                return str(project.project_id)
        page_token = str(response.next_page_token or "") or None
        if not page_token:
            break

    say(f"Creating project {name!r}")
    created = create_project.sync(
        client=client,
        body=CreateProjectInput(name=name, description=PROJECT_DESCRIPTION),
    )
    assert created is not None, f"failed to create project {name!r}"
    return str(created.project_id)


# A light batch never runs a container under this system, so these build
# defaults are never actually used; the API requires them regardless.
_UNUSED_BUILD_RESOURCES = {
    "build_vcpus": 1,
    "build_memory_mib": 1024,
    "build_gpus": 0,
    "build_shared_memory_mb": 64,
    "metrics_build_vcpus": 1,
    "metrics_build_memory_mib": 1024,
    "metrics_build_gpus": 0,
    "metrics_build_shared_memory_mb": 64,
}


def resolve_system(
    client: AuthenticatedClient,
    project_id: str,
    name: str,
    say: Any = lambda _: None,
) -> str:
    """Return the ID of the system called ``name``, creating it if needed."""
    response = list_systems.sync_detailed(project_id, client=client, name=name)
    assert response.parsed is not None, f"failed to list systems: {response.content!r}"
    for system in response.parsed.systems or []:
        if system.name == name:
            return str(system.system_id)

    say(f"Creating system {name!r}")
    created = create_system.sync_detailed(
        project_id,
        client=client,
        body=CreateSystemInput(
            name=name,
            description=PROJECT_DESCRIPTION,
            architecture=Architecture.AMD64,
            **_UNUSED_BUILD_RESOURCES,
        ),
    )
    assert created.parsed is not None, (
        f"failed to create system {name!r}: {created.content!r}"
    )
    return str(created.parsed.system_id)


def resolve_experience_tag(
    client: AuthenticatedClient,
    project_id: str,
    name: str,
    say: Any = lambda _: None,
) -> str:
    """Return the ID of the experience tag called ``name``, creating it if needed."""
    page_token: Optional[str] = None
    while True:
        kwargs: dict[str, Any] = {"client": client, "name": name}
        if page_token:
            kwargs["page_token"] = page_token
        response = list_experience_tags.sync_detailed(project_id, **kwargs)
        assert response.parsed is not None, (
            f"failed to list experience tags: {response.content!r}"
        )
        for tag in response.parsed.experience_tags or []:
            if tag.name == name:
                return str(tag.experience_tag_id)
        page_token = str(response.parsed.next_page_token or "") or None
        if not page_token:
            break

    say(f"Creating experience tag {name!r}")
    created = create_experience_tag.sync_detailed(
        project_id,
        client=client,
        body=CreateExperienceTagInput(name=name, description=PROJECT_DESCRIPTION),
    )
    assert created.parsed is not None, (
        f"failed to create experience tag {name!r}: {created.content!r}"
    )
    return str(created.parsed.experience_tag_id)


def resolve_experience(
    client: AuthenticatedClient,
    project_id: str,
    name: str,
    say: Any = lambda _: None,
) -> str:
    """Return the ID of the experience called ``name``, creating it if needed."""
    response = list_experiences.sync_detailed(project_id, client=client, name=name)
    assert response.parsed is not None, (
        f"failed to list experiences: {response.content!r}"
    )
    for experience in response.parsed.experiences or []:
        if experience.name == name:
            return str(experience.experience_id)

    say(f"Creating experience {name!r}")
    created = create_experience.sync_detailed(
        project_id,
        client=client,
        body=CreateExperienceInput(name=name, description=PROJECT_DESCRIPTION),
    )
    assert created.parsed is not None, (
        f"failed to create experience {name!r}: {created.content!r}"
    )
    return str(created.parsed.experience_id)


def resolve_test_suite(
    client: AuthenticatedClient,
    project_id: str,
    name: str,
    system_id: str,
    experience_ids: list[str],
    metrics_set: str,
    say: Any = lambda _: None,
) -> TestSuite:
    """Return the test suite called ``name``, holding exactly ``experience_ids``.

    Creates it if missing. An existing suite whose experiences or metrics set
    differ is revised to match, so re-running the demo after its data changes
    does not leave batches attached to a stale suite.
    """
    response = list_test_suites.sync_detailed(project_id, client=client, name=name)
    assert response.parsed is not None, (
        f"failed to list test suites: {response.content!r}"
    )
    existing = next(
        (suite for suite in response.parsed.test_suites or [] if suite.name == name),
        None,
    )
    if existing is None:
        say(f"Creating test suite {name!r}")
        created = create_test_suite.sync_detailed(
            project_id,
            client=client,
            body=CreateTestSuiteInput(
                name=name,
                description=PROJECT_DESCRIPTION,
                system_id=system_id,
                experiences=experience_ids,
                metrics_set_name=metrics_set,
            ),
        )
        assert created.parsed is not None, (
            f"failed to create test suite {name!r}: {created.content!r}"
        )
        return created.parsed

    if (
        set(existing.experiences) == set(experience_ids)
        and existing.metrics_set_name == metrics_set
    ):
        return existing

    say(f"Revising test suite {name!r}")
    revised = revise_test_suite.sync_detailed(
        project_id,
        existing.test_suite_id,
        client=client,
        body=ReviseTestSuiteInput(
            update_metrics_build=False,
            experiences=experience_ids,
            metrics_set_name=metrics_set,
        ),
    )
    assert revised.parsed is not None, (
        f"failed to revise test suite {name!r}: {revised.content!r}"
    )
    return revised.parsed


def tag_experience(
    client: AuthenticatedClient, project_id: str, tag_id: str, experience_id: str
) -> None:
    """Attach an experience tag to an experience.

    A 409 means it is already tagged, which is the state this is asking for,
    not a failure - re-running the demo against the same project hits this on
    every pass after the first.
    """
    response = add_experience_tag_to_experience.sync_detailed(
        project_id, tag_id, experience_id, client=client
    )
    if response.status_code not in (200, 201, 204, 409):
        raise Exception(
            f"failed to tag experience {experience_id} with {tag_id}: "
            f"{response.status_code} {response.content!r}"
        )


def replay_job(
    client: AuthenticatedClient,
    batch: Batch,
    data: Bundle,
    job: dict[str, Any],
) -> Test:
    """Replay one captured job into a new test, uploaded but not yet closed.

    Emissions are replayed through the typed ``Test`` methods rather than by
    uploading the captured file verbatim, so they are validated against the
    demo's config on the way through. If the data and the config ever drift
    apart, that fails here rather than showing up as an empty chart.

    The caller closes the returned test once every test has been uploaded.
    """
    job_dir = data.root / str(job["directory"])
    test = Test(client, batch, str(job["experience_name"]))
    for file_name, log_type in _attachments(job):
        test.attach_log(str(job_dir / file_name), log_type)
    for topic, payload, timestamp, is_event in _emissions(
        job_dir / str(job["emissions"])
    ):
        if is_event and timestamp is not None:
            test.emit_event(topic, payload, timestamp)
        elif timestamp is not None:
            test.emit(topic, payload, timestamp)
        else:
            test.emit(topic, payload)

    scratch = Path(test.output_path)
    test.upload_emissions()
    # Test leaves its emissions file in the working directory; one per test
    # would litter the directory the demo was run from.
    scratch.unlink(missing_ok=True)
    return test


def _attachments(job: dict[str, Any]) -> list[tuple[str, Optional[LogType]]]:
    """The files to upload alongside a job's emissions, and how to type each.

    The log type decides what ReSim can do with a file — an ``.mcap`` sent as
    ``FOXGLOVE_MCAP_LOG`` opens in the viewer, where the same bytes sent as
    something else are only a download — so the bundle records the type its
    source batch used rather than leaving it to be guessed from the name.

    ``media`` is the older bundle shape, which carried filenames alone.
    """
    attachments: list[tuple[str, Optional[LogType]]] = [
        (str(entry["file"]), _log_type(entry.get("log_type")))
        for entry in job.get("artifacts") or []
    ]
    known = {file_name for file_name, _ in attachments}
    attachments.extend(
        (str(file_name), None)
        for file_name in job.get("media") or []
        if str(file_name) not in known
    )
    return attachments


def _log_type(name: Optional[str]) -> Optional[LogType]:
    """Parse a log type from the manifest, ignoring one this SDK does not know.

    A bundle can name a type added after this version shipped; falling back to
    None lets ReSim infer from the filename rather than failing the run.
    """
    if not name:
        return None
    try:
        return LogType(name)
    except ValueError:
        return None


def _emissions(
    path: Path,
) -> Iterator[tuple[str, dict[str, Any], Optional[int], bool]]:
    """Yield ``(topic, data, timestamp, is_event)`` for each line in a JSONL file."""
    if not path.is_file():
        raise DemoDataError(f"missing emissions file {path} in the demo data bundle")
    with open(path, "r", encoding="utf8") as f:
        for number, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
                metadata = record["$metadata"]
                yield (
                    str(metadata["topic"]),
                    record["$data"],
                    metadata.get("timestamp"),
                    bool(metadata.get("event")),
                )
            except (json.JSONDecodeError, KeyError, TypeError) as e:
                raise DemoDataError(
                    f"malformed emission at {path}:{number}: {e}"
                ) from e


def config_path(demo: str) -> Path:
    """Return the path to the metrics config a demo runs with.

    Copy it as the starting point for your own config, or read it to see how
    the charts in the demo are defined::

        from signalflag.demo import config_path

        print(config_path("navigation").read_text())
        print(config_path("mujoco").read_text())

    Args:
        demo: Which demo's config to return. See :data:`DEMOS`.

    Returns:
        A real filesystem path. The templates the config references live in
        :func:`templates_path`.
    """
    return _materialise(_package_data(get_demo(demo))[0])


def templates_path(demo: str) -> Path:
    """Return the path to the ``.liquid`` templates a demo's config uses.

    Args:
        demo: Which demo's templates to return. See :data:`DEMOS`.

    Returns:
        A real filesystem path to the directory holding them.
    """
    return _materialise(_package_data(get_demo(demo))[1])


def _package_data(demo: Demo) -> tuple[Any, Any]:
    """Locate a demo's shipped metrics config and template directory."""
    root = resources.files("signalflag.demo") / "data"
    return root / demo.config_file, root / "templates"


def _materialise(resource: Any) -> Path:
    """Return a real path for package data, copying it out if it is not one.

    An ordinary install puts package data on disk, so this is the resource
    itself. Zip imports have no such path, so the data is copied into the same
    cache the demo bundle uses, where it stays valid after this returns.
    """
    if isinstance(resource, Path):
        return resource

    target: Path = bundle.cache_dir() / "package-data" / str(resource.name)
    if not target.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        with resources.as_file(resource) as source:
            if source.is_dir():
                shutil.copytree(source, target, dirs_exist_ok=True)
            else:
                shutil.copy2(source, target)
    return target


def _urls(
    client: AuthenticatedClient,
    project_id: str,
    batch_ids: dict[str, str],
    dashboard_id: Optional[str],
    job_ids: Optional[dict[str, str]] = None,
    sessions: bool = False,
    has_dashboard: bool = True,
    suite: Optional[TestSuite] = None,
) -> dict[str, str]:
    app = links.app_base_url(client.get_httpx_client()._base_url)
    job_ids = job_ids or {}
    urls = {
        f"batch_{side}": (
            links.job_url(app, project_id, batch_id, job_ids[side])
            if side in job_ids
            else links.batch_url(app, project_id, batch_id)
        )
        for side, batch_id in batch_ids.items()
    }
    # "Compare" only means something for a pair; three or more batches trend
    # on the dashboard instead of comparing pairwise.
    if len(batch_ids) == 2:
        first, second = batch_ids.values()
        urls["compare"] = links.compare_batches_url(app, project_id, first, second)
    # A demo with no dashboard config has nothing to point at - not even the
    # dashboards list, which would just be empty or someone else's.
    if has_dashboard:
        urls["dashboard"] = (
            links.dashboard_url(app, project_id, dashboard_id)
            if dashboard_id
            else links.dashboards_url(app, project_id)
        )
    if sessions:
        urls["sessions"] = links.sessions_url(app, project_id)
    if suite is not None:
        urls["test_suite"] = links.test_suite_url(
            app, project_id, str(suite.test_suite_id), suite.test_suite_revision
        )
    return urls


def _report(
    urls: dict[str, str], sides: tuple[str, ...], versions: dict[str, str]
) -> None:
    labels = []
    if "sessions" in urls:
        # The main thing to look at for a sessions-style demo; individual
        # batch links below are for digging into one session's raw results.
        labels.append(("sessions", "Sessions view"))
    if len(sides) == 2:
        first, second = sides
        labels.append((f"batch_{first}", f"Batch A (baseline, {versions[first]})"))
        labels.append((f"batch_{second}", f"Batch B (candidate, {versions[second]})"))
        labels.append(("compare", "A/B comparison"))
    else:
        for side in sides:
            version = versions[side]
            suffix = f" ({version})" if version else ""
            labels.append((f"batch_{side}", f"Batch {side}{suffix}"))
    if "dashboard" in urls:
        labels.append(("dashboard", "Trends dashboard"))
    if "test_suite" in urls:
        labels.append(("test_suite", "Test suite"))

    width = max(len(label) for _, label in labels)
    print("\nSignalFlag SDK demo complete.\n")
    for key, label in labels:
        print(f"  {label.ljust(width)}   {urls[key]}")
    print(
        "\nMetrics are computed after each batch closes, which takes a few "
        "minutes.\nReload these pages when it finishes."
    )
