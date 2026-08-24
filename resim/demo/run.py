# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Run the ReSim SDK demo: two comparable batches, and links to the results."""

import json
import os
import shutil
from dataclasses import dataclass
from importlib import resources
from pathlib import Path
from typing import Any, Iterator, Optional, Union

from httpx import URL

from resim.demo import bundle, links
from resim.demo.bundle import Bundle, DemoDataError
from resim.sdk.auth.const import (
    DEFAULT_BASE_URL,
    DEFAULT_CACHE_LOCATION,
    DEFAULT_DOMAIN,
)
from resim.sdk.auth.device_code_client import DEVICE_CODE_CLIENT_ID, DeviceCodeClient
from resim.sdk.batch import Batch
from resim.sdk.bff_client.dashboards import find_dashboard_id
from resim.sdk.client import AuthenticatedClient
from resim.sdk.client.api.projects import create_project, list_projects
from resim.sdk.client.models.create_project_input import CreateProjectInput
from resim.sdk.test import Test

__all__ = ["DEMOS", "Demo", "DemoResult", "config_path", "run", "templates_path"]

PROJECT_DESCRIPTION = "Created by the ReSim Python SDK demo (resim.demo.run)."


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
    #: Metrics config shipped in ``resim/demo/data``.
    config_file: str
    metrics_set: str
    dashboard_name: str


DEMOS: dict[str, Demo] = {
    "navigation": Demo(
        key="navigation",
        summary=(
            "A hospital navigation suite: dense telemetry across 34 scenarios, "
            "covering every chart type ReSim ships."
        ),
        project_name="ReSim SDK Demo",
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
            "A bimanual manipulation policy in MuJoCo: one test per seed, "
            "compared across two policy builds."
        ),
        project_name="ReSim SDK Demo (MuJoCo)",
        branch="sdk-demo-mujoco",
        bundle=bundle.BundleSource(
            url=(
                "https://resim-public-assets.s3.us-east-1.amazonaws.com"
                "/sdk-demo/resim-sdk-demo-mujoco-v1.tar.gz"
            ),
            sha256="f5bb2f84808fc5fad2253e3b73a2f51e0e718adfc314dca3567d5f8181f8479d",
            cache_key="mujoco-v1",
        ),
        config_file="mujoco.resim.yml",
        metrics_set="MuJoCo Metrics",
        dashboard_name="MuJoCo Demo Trends",
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
# them, but we do, to smoke-test against staging.
ENV_API_URL = "RESIM_API_URL"
ENV_AUTH_DOMAIN = "RESIM_AUTH_DOMAIN"
ENV_CLIENT_ID = "RESIM_CLIENT_ID"

# Both batches run on the same branch: a dashboard is scoped to one branch, so
# this is what lets a single dashboard trend across both of them. They are told
# apart by their build version instead.
SIDES = ("a", "b")


@dataclass(frozen=True)
class DemoResult:
    """Where the demo put its results."""

    project_id: str
    batch_ids: dict[str, str]
    dashboard_id: Optional[str]
    urls: dict[str, str]


def run(
    project_name: Optional[str] = None,
    *,
    demo: str,
    client: Optional[AuthenticatedClient] = None,
    branch: Optional[str] = None,
    data_dir: Optional[Union[str, Path]] = None,
    quiet: bool = False,
) -> DemoResult:
    """Populate a ReSim project with two comparable batches and print the links.

    Creates the project if it does not exist, syncs the demo's metrics config
    (which also creates the branch and the trends dashboard), then replays real
    test data into two light batches that differ only by build version. The
    result is a batch to look at, a second batch to compare it against, and a
    dashboard that trends both.

    Args:
        project_name: Project to run in. Created if it does not exist.
            Defaults to the chosen demo's own project name.
        demo: Which demo to run. See :data:`DEMOS`.
        client: An authenticated ReSim API client. Defaults to interactive
            device code authentication against production.
        branch: Branch to create the batches on. Both batches share it.
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
    data = bundle.ensure(chosen.bundle, data_dir)

    if client is None:
        client = default_client()

    project_id = resolve_project(client, project_name, say)

    config_resource, templates_resource = _package_data(chosen)

    batch_ids: dict[str, str] = {}
    dashboard_id: Optional[str] = None
    with (
        resources.as_file(config_resource) as config,
        resources.as_file(templates_resource) as templates,
    ):
        for side in SIDES:
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
            ) as batch:
                batch_ids[side] = batch.id
                if dashboard_id is None:
                    dashboard_id = find_dashboard_id(
                        client,
                        project_id,
                        batch.branch_id,
                        chosen.dashboard_name,
                    )
                tests: list[Test] = []
                try:
                    for job in jobs:
                        tests.append(replay_job(client, batch, data, job))
                        say(f"  {job.get('experience_name')}")
                finally:
                    for test in tests:
                        test.close()

    urls = _urls(client, project_id, batch_ids, dashboard_id)
    if not quiet:
        _report(
            urls, {side: str(data.batch(side).get("version") or "") for side in SIDES}
        )

    return DemoResult(
        project_id=project_id,
        batch_ids=batch_ids,
        dashboard_id=dashboard_id,
        urls=urls,
    )


def default_client() -> AuthenticatedClient:
    """Authenticate interactively, honouring the deployment env overrides."""
    base_url = os.environ.get(ENV_API_URL)
    domain = os.environ.get(ENV_AUTH_DOMAIN)
    client_id = os.environ.get(ENV_CLIENT_ID)
    if not (base_url or domain or client_id):
        return DeviceCodeClient()

    # Tokens from different deployments are not interchangeable, so give a
    # non-production login its own cache rather than overwriting the one the
    # user's production sessions rely on.
    cache = DEFAULT_CACHE_LOCATION
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
    for file_name in job.get("media") or []:
        test.attach_log(str(job_dir / file_name))
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

        from resim.demo import config_path

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
    root = resources.files("resim.demo") / "data"
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
) -> dict[str, str]:
    app = links.app_base_url(client.get_httpx_client()._base_url)
    urls = {
        "batch_a": links.batch_url(app, project_id, batch_ids["a"]),
        "batch_b": links.batch_url(app, project_id, batch_ids["b"]),
        "compare": links.compare_batches_url(
            app, project_id, batch_ids["a"], batch_ids["b"]
        ),
    }
    urls["dashboard"] = (
        links.dashboard_url(app, project_id, dashboard_id)
        if dashboard_id
        else links.dashboards_url(app, project_id)
    )
    return urls


def _report(urls: dict[str, str], versions: dict[str, str]) -> None:
    labels = (
        ("batch_a", f"Batch A (baseline, {versions['a']})"),
        ("batch_b", f"Batch B (candidate, {versions['b']})"),
        ("compare", "A/B comparison"),
        ("dashboard", "Trends dashboard"),
    )
    width = max(len(label) for _, label in labels)
    print("\nReSim SDK demo complete.\n")
    for key, label in labels:
        print(f"  {label.ljust(width)}   {urls[key]}")
    print(
        "\nMetrics are computed after each batch closes, which takes a few "
        "minutes.\nReload these pages when it finishes."
    )
