# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Orchestration tests for the demo, with the API stubbed out.

The point of these is the parts that are ours: that both batches run on one
branch with matching test names and differing versions, that the printed links
are right, and that every emission the demo replays validates against the
config it ships. Emission validation runs for real - the fake test object is an
``Emitter`` pointed at the shipped config - so data and config drifting apart
fails here.
"""

import json
import os
import tempfile
import unittest
from importlib import import_module, resources
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

from httpx import URL

from resim.demo.bundle import DemoDataError
from resim.demo.run import DASHBOARD_NAME, METRICS_SET, resolve_project, run
from resim.sdk.metrics.emissions import Emitter

# resim.demo exports a `run` function, which shadows the `resim.demo.run`
# module attribute, so the module has to be fetched by name to patch into it.
run_module = import_module("resim.demo.run")

EXPERIENCES = ["Corridor - Bright", "Corridor - Dark"]

CONFIG_PATH = resources.files("resim.demo") / "data" / "config.resim.yml"


def _emissions(with_media: bool) -> list[dict[str, Any]]:
    """A small run covering scalar, series, media, and event topics."""
    records: list[dict[str, Any]] = [
        {"$metadata": {"topic": "goal_count"}, "$data": {"count": 2}},
        {
            "$metadata": {"topic": "goal_status", "timestamp": 0},
            "$data": {"state": "Navigating: Goal 1"},
        },
        {
            "$metadata": {"topic": "robot_trajectory"},
            "$data": {"raw_metric": '{"data": [], "layout": {}}'},
        },
        {
            "$metadata": {
                "topic": "goal_reached",
                "timestamp": 5_000_000_000,
                "event": True,
            },
            "$data": {
                "name": "Goal 1 reached",
                "description": "Arrived at the first goal",
                "status": "PASSED",
                "tags": ["navigation"],
            },
        },
    ]
    for i in range(3):
        records.append(
            {
                "$metadata": {"topic": "odom_linear_velocity", "timestamp": i * 10**9},
                "$data": {"x": 0.4 + i * 0.1, "y": 0.0, "z": 0.0},
            }
        )
        records.append(
            {
                "$metadata": {"topic": "pose_difference", "timestamp": i * 10**9},
                "$data": {"position_diff_m": 0.05 * i},
            }
        )
        records.append(
            {
                "$metadata": {"topic": "time_to_goal"},
                "$data": {"goal_name": f"Goal {i}", "time_s": 12.5},
            }
        )
    if with_media:
        records.append(
            {
                "$metadata": {"topic": "camera_video"},
                "$data": {"camera_name": "front", "filename": "camera.mp4"},
            }
        )
        records.append(
            {
                "$metadata": {"topic": "camera_frame"},
                "$data": {"camera_name": "front", "filename": "camera_frame.jpg"},
            }
        )
    return records


def write_fixture_bundle(root: Path) -> None:
    """Write a two-experience-per-side bundle in the shape build_bundle emits."""
    batches: dict[str, Any] = {}
    for side, name, version in (
        ("a", "Nav stack v2 (baseline)", "nav-v2.0.0"),
        ("b", "Nav stack v3 (candidate)", "nav-v3.0.0"),
    ):
        jobs = []
        for index, experience in enumerate(EXPERIENCES):
            directory = f"{side}/{experience.replace(' ', '-')}"
            job_dir = root / directory
            job_dir.mkdir(parents=True, exist_ok=True)
            with_media = index == 0
            (job_dir / "emissions.resim.jsonl").write_text(
                "".join(json.dumps(r) + "\n" for r in _emissions(with_media)),
                encoding="utf8",
            )
            media = []
            if with_media:
                (job_dir / "camera.mp4").write_bytes(b"fake mp4")
                (job_dir / "camera_frame.jpg").write_bytes(b"fake jpg")
                media = ["camera.mp4", "camera_frame.jpg"]
            jobs.append(
                {
                    "experience_name": experience,
                    "directory": directory,
                    "emissions": "emissions.resim.jsonl",
                    "media": media,
                }
            )
        batches[side] = {"name": name, "version": version, "jobs": jobs}
    (root / "manifest.json").write_text(
        json.dumps({"version": 1, "batches": batches}), encoding="utf8"
    )


class FakeBatch:
    """Stands in for resim.sdk.batch.Batch, recording how it was constructed."""

    created: list[dict[str, Any]] = []
    closed: list[str] = []

    def __init__(self, **kwargs: Any) -> None:
        self.kwargs = kwargs
        self.id = f"batch-{len(FakeBatch.created)}"
        self.branch_id = "branch-1"
        self.project_id = kwargs.get("project_id")
        self.metrics_config_path = kwargs.get("metrics_config_path")

    def __enter__(self) -> "FakeBatch":
        FakeBatch.created.append(self.kwargs)
        return self

    def __exit__(self, *args: Any) -> None:
        FakeBatch.closed.append(self.id)


class FakeTest(Emitter):
    """Stands in for resim.sdk.test.Test, validating against the shipped config."""

    started: list[tuple[str, str]] = []
    attached: list[str] = []
    events: list[tuple[str, str, str]] = []
    output_dir: Path = Path()

    def __init__(self, client: Any, batch: Any, name: str) -> None:
        self.name = name
        self.batch_id = batch.id
        self._closed = False
        FakeTest.started.append((batch.id, name))
        output = FakeTest.output_dir / f"{batch.id}-{len(FakeTest.started)}.jsonl"
        super().__init__(config_path=batch.metrics_config_path, output_path=output)

    def attach_log(self, file_path: str, *args: Any, **kwargs: Any) -> None:
        assert Path(file_path).is_file(), f"{file_path} does not exist"
        FakeTest.attached.append(Path(file_path).name)

    def upload_emissions(self) -> None:
        if self.file is None:
            return
        FakeTest.events.append((self.batch_id, "upload", self.name))
        Emitter.close(self)

    def close(self, *args: Any, **kwargs: Any) -> None:
        if self._closed:
            return
        self.upload_emissions()
        self._closed = True
        FakeTest.events.append((self.batch_id, "close", self.name))

    def __enter__(self) -> "FakeTest":
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()


def fake_client() -> MagicMock:
    client = MagicMock()
    httpx_client = MagicMock()
    httpx_client._base_url = URL("https://api.resim.ai/v1/")
    client.get_httpx_client.return_value = httpx_client
    return client


class RunTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        write_fixture_bundle(self.root)
        FakeBatch.created = []
        FakeBatch.closed = []
        FakeTest.started = []
        FakeTest.attached = []
        FakeTest.events = []
        FakeTest.output_dir = self.root / "emitted"
        FakeTest.output_dir.mkdir()

        self.patches: list[Any] = [
            patch.object(run_module, "Batch", FakeBatch),
            patch.object(run_module, "Test", FakeTest),
            patch.object(run_module, "resolve_project", return_value="project-1"),
            patch.object(run_module, "find_dashboard_id", return_value="dash-1"),
        ]
        for p in self.patches:
            p.start()

    def tearDown(self) -> None:
        for p in self.patches:
            p.stop()
        self.temp.cleanup()

    def _run(self) -> Any:
        return run(client=fake_client(), data_dir=self.root, quiet=True)

    def test_runs_both_batches(self) -> None:
        result = self._run()
        self.assertEqual(len(FakeBatch.created), 2)
        self.assertEqual(sorted(result.batch_ids), ["a", "b"])

    def test_both_batches_close(self) -> None:
        self._run()
        self.assertEqual(len(FakeBatch.closed), 2)

    def test_batches_share_a_branch_and_metrics_set(self) -> None:
        self._run()
        branches = {kwargs["branch"] for kwargs in FakeBatch.created}
        self.assertEqual(len(branches), 1, "one dashboard cannot span two branches")
        self.assertEqual(
            {kwargs["metrics_set_name"] for kwargs in FakeBatch.created}, {METRICS_SET}
        )

    def test_batches_differ_by_version(self) -> None:
        self._run()
        versions = [kwargs["version"] for kwargs in FakeBatch.created]
        self.assertEqual(versions, ["nav-v2.0.0", "nav-v3.0.0"])
        self.assertEqual(len(set(versions)), 2, "the A/B cards need these to differ")

    def test_both_batches_run_the_same_experiences(self) -> None:
        self._run()
        per_batch: dict[str, list[str]] = {}
        for batch_id, name in FakeTest.started:
            per_batch.setdefault(batch_id, []).append(name)
        self.assertEqual(len(per_batch), 2)
        first, second = per_batch.values()
        self.assertEqual(
            sorted(first), sorted(second), "tests pair by name in the A/B view"
        )
        self.assertEqual(sorted(first), sorted(EXPERIENCES))

    def test_ships_the_config_and_templates_to_each_batch(self) -> None:
        self._run()
        for kwargs in FakeBatch.created:
            self.assertTrue(Path(kwargs["metrics_config_path"]).is_file())
            self.assertTrue(Path(kwargs["templates_path"]).is_dir())
            self.assertTrue(
                list(Path(kwargs["templates_path"]).glob("*.liquid")),
                "custom templates must reach the batch",
            )

    def test_attaches_media_where_the_manifest_lists_it(self) -> None:
        self._run()
        self.assertEqual(
            sorted(FakeTest.attached),
            ["camera.mp4", "camera.mp4", "camera_frame.jpg", "camera_frame.jpg"],
        )

    def test_emissions_validate_against_the_shipped_config(self) -> None:
        # FakeTest is an Emitter built from the shipped config, so a topic or
        # field the config does not declare raises during replay.
        self._run()

    def test_urls_point_at_the_app(self) -> None:
        urls = self._run().urls
        self.assertEqual(
            urls["batch_a"], "https://app.resim.ai/projects/project-1/batches/batch-0"
        )
        self.assertEqual(
            urls["compare"],
            "https://app.resim.ai/projects/project-1/batches/batch-0"
            "/compare-batch/batch/batch-1",
        )
        self.assertEqual(
            urls["dashboard"],
            "https://app.resim.ai/projects/project-1/dashboards/dash-1",
        )

    def test_looks_the_dashboard_up_by_name(self) -> None:
        self._run()
        run_module.find_dashboard_id.assert_called_with(
            unittest.mock.ANY, "project-1", "branch-1", DASHBOARD_NAME
        )

    def test_falls_back_to_the_dashboards_list_when_not_found(self) -> None:
        with patch.object(run_module, "find_dashboard_id", return_value=None):
            result = self._run()
        self.assertIsNone(result.dashboard_id)
        self.assertEqual(
            result.urls["dashboard"],
            "https://app.resim.ai/projects/project-1/dashboards",
        )

    def test_prints_every_link(self) -> None:
        with patch("builtins.print") as printed:
            result = run(client=fake_client(), data_dir=self.root)
        output = "\n".join(
            str(call.args[0]) for call in printed.call_args_list if call.args
        )
        for url in result.urls.values():
            self.assertIn(url, output)

    def test_missing_emissions_file_is_reported(self) -> None:
        for path in self.root.rglob("emissions.resim.jsonl"):
            path.unlink()
        with self.assertRaises(DemoDataError) as ctx:
            self._run()
        self.assertIn("emissions", str(ctx.exception))

    def test_authenticates_for_the_caller_when_no_client_is_given(self) -> None:
        # The library form takes an optional client; the CLI relies on this
        # branch to authenticate interactively.
        with patch.object(
            run_module, "default_client", return_value=fake_client()
        ) as default:
            run(data_dir=self.root, quiet=True)
        default.assert_called_once_with()

    def test_blank_lines_in_the_emissions_file_are_skipped(self) -> None:
        # Trailing newlines are normal in JSONL; they are not malformed data.
        for path in self.root.rglob("emissions.resim.jsonl"):
            path.write_text(path.read_text() + "\n\n", encoding="utf8")

        self._run()

    def test_a_batch_uploads_every_test_before_closing_any(self) -> None:
        # Closing a job right after creating it races the scheduler's own first
        # transition of its tasks, which strands the batch.
        self._run()

        for batch_id in {e[0] for e in FakeTest.events}:
            actions = [e[1] for e in FakeTest.events if e[0] == batch_id]
            self.assertEqual(
                actions,
                ["upload"] * len(EXPERIENCES) + ["close"] * len(EXPERIENCES),
                f"batch {batch_id} interleaved uploads and closes",
            )

    def test_tests_are_closed_even_if_a_later_one_fails(self) -> None:
        # A failure partway through must not leave jobs open.
        real = run_module.replay_job
        calls = {"n": 0}

        def flaky(*args: Any, **kwargs: Any) -> Any:
            calls["n"] += 1
            if calls["n"] == 2:
                raise RuntimeError("boom")
            return real(*args, **kwargs)

        with patch.object(run_module, "replay_job", side_effect=flaky):
            with self.assertRaises(RuntimeError):
                self._run()

        uploaded = [e[2] for e in FakeTest.events if e[1] == "upload"]
        closed = [e[2] for e in FakeTest.events if e[1] == "close"]
        self.assertTrue(uploaded)
        self.assertEqual(closed, uploaded)

    def test_leaves_no_emissions_files_behind(self) -> None:
        # Test writes its emissions next to the working directory; the demo is
        # responsible for not leaving 70-odd of them lying around.
        self._run()
        self.assertEqual(
            [p.name for p in FakeTest.output_dir.iterdir()],
            [],
            "replayed emissions files should be cleaned up",
        )

    def test_malformed_emission_is_reported_with_its_line(self) -> None:
        path = next(self.root.rglob("emissions.resim.jsonl"))
        path.write_text("{not json}\n", encoding="utf8")
        with self.assertRaises(DemoDataError) as ctx:
            self._run()
        self.assertIn(":1", str(ctx.exception))


class DefaultClientTest(unittest.TestCase):
    """The env overrides decide which deployment is hit and which token cache is
    written, so both are pinned: a staging login must not overwrite the token a
    production session relies on."""

    def setUp(self) -> None:
        for key in (
            run_module.ENV_API_URL,
            run_module.ENV_AUTH_DOMAIN,
            run_module.ENV_CLIENT_ID,
        ):
            os.environ.pop(key, None)

    def test_no_overrides_uses_the_sdk_defaults(self) -> None:
        with patch.object(run_module, "DeviceCodeClient") as client:
            run_module.default_client()
        client.assert_called_once_with()

    def test_api_url_override_is_passed_through(self) -> None:
        os.environ[run_module.ENV_API_URL] = "https://api.example.io/v1"
        with patch.object(run_module, "DeviceCodeClient") as client:
            run_module.default_client()
        self.assertEqual(
            client.call_args.kwargs["base_url"], "https://api.example.io/v1"
        )

    def test_a_non_default_deployment_gets_its_own_token_cache(self) -> None:
        os.environ[run_module.ENV_API_URL] = "https://api.example.io/v1"
        with patch.object(run_module, "DeviceCodeClient") as client:
            run_module.default_client()

        cache = client.call_args.kwargs["cache_location"]
        self.assertEqual(cache.name, "token-api.example.io.json")
        self.assertNotEqual(cache, run_module.DEFAULT_CACHE_LOCATION)

    def test_unset_overrides_fall_back_to_the_defaults(self) -> None:
        os.environ[run_module.ENV_CLIENT_ID] = "some-client"
        with patch.object(run_module, "DeviceCodeClient") as client:
            run_module.default_client()

        kwargs = client.call_args.kwargs
        self.assertEqual(kwargs["client_id"], "some-client")
        self.assertEqual(kwargs["base_url"], run_module.DEFAULT_BASE_URL)
        self.assertEqual(kwargs["domain"], run_module.DEFAULT_DOMAIN)


class ResolveProjectTest(unittest.TestCase):
    def setUp(self) -> None:
        self.client = fake_client()

    def _projects(self, *names: str) -> MagicMock:
        projects = []
        for n in names:
            # `name` is reserved by the MagicMock constructor, so set it after.
            project = MagicMock(project_id=f"id-{n}")
            project.name = n
            projects.append(project)
        response = MagicMock()
        response.projects = projects
        response.next_page_token = None
        return response

    def test_returns_an_existing_project(self) -> None:
        with patch.object(
            run_module.list_projects, "sync", return_value=self._projects("mine")
        ):
            self.assertEqual(resolve_project(self.client, "mine"), "id-mine")

    def test_creates_a_missing_project(self) -> None:
        created = MagicMock(project_id="new-id")
        with (
            patch.object(
                run_module.list_projects, "sync", return_value=self._projects("other")
            ),
            patch.object(
                run_module.create_project, "sync", return_value=created
            ) as create,
        ):
            self.assertEqual(resolve_project(self.client, "mine"), "new-id")
        self.assertEqual(create.call_args.kwargs["body"].name, "mine")

    def test_pages_through_projects(self) -> None:
        first = self._projects("other")
        first.next_page_token = "page-2"
        second = self._projects("mine")
        with patch.object(
            run_module.list_projects, "sync", side_effect=[first, second]
        ) as listed:
            self.assertEqual(resolve_project(self.client, "mine"), "id-mine")
        self.assertEqual(listed.call_count, 2)


if __name__ == "__main__":
    unittest.main()
