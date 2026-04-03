# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import base64
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

import yaml
from httpx import URL

from resim.sdk.bff_client.metrics import read_templates, sync_config


class SyncConfigTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.config_path = Path(self.temp_dir.name) / "config.resim.yml"
        self.config_contents = b"metrics_config: true\n"
        self.config_path.write_bytes(self.config_contents)

        self.mock_response = MagicMock()
        self.mock_response.status_code = 200
        self.mock_response.json.return_value = {"data": {"updateMetricsConfig": None}}

        self.mock_httpx = MagicMock()
        self.mock_httpx._base_url = URL("https://api.resim.ai/v1/")
        self.mock_httpx.post.return_value = self.mock_response

        self.mock_client = MagicMock()
        self.mock_client.get_httpx_client.return_value = self.mock_httpx

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def test_metrics_sync(self) -> None:
        sync_config(
            client=self.mock_client,
            project_id="proj-123",
            branch_name="main",
            config_path=str(self.config_path),
        )

        self.mock_httpx.post.assert_called_once()
        call_kwargs = self.mock_httpx.post.call_args

        # Verify BFF URL host substitution
        posted_url = call_kwargs.args[0]
        self.assertIn("bff.resim.ai", posted_url)

        # Verify body payload
        body = call_kwargs.kwargs["json"]
        variables = body["variables"]
        self.assertEqual(variables["projectId"], "proj-123")
        self.assertEqual(variables["branch"], "main")
        self.assertEqual(variables["templateFiles"], [])  # no templates_path → empty
        self.assertEqual(
            variables["config"],
            base64.b64encode(self.config_contents).decode(),
        )

    def test_metrics_sync_multiple_paths_merged(self) -> None:
        path2 = Path(self.temp_dir.name) / "config2.resim.yml"
        self.config_path.write_text(
            "version: 1\ntopics:\n  t1:\n    schema:\n      a: float\n",
            encoding="utf8",
        )
        path2.write_text(
            "version: 1\ntopics:\n  t2:\n    schema:\n      b: int\n",
            encoding="utf8",
        )
        sync_config(
            client=self.mock_client,
            project_id="proj-123",
            branch_name="main",
            config_path=[str(self.config_path), str(path2)],
        )

        self.mock_httpx.post.assert_called_once()
        body = self.mock_httpx.post.call_args.kwargs["json"]
        variables = body["variables"]
        decoded = base64.b64decode(variables["config"])
        loaded = yaml.safe_load(decoded)
        self.assertEqual(loaded["version"], 1)
        self.assertEqual(set(loaded["topics"].keys()), {"t1", "t2"})
        self.assertEqual(loaded["metrics"], {})
        self.assertEqual(loaded["metrics sets"], {})

    def test_raises_on_non_200_status(self) -> None:
        self.mock_response.status_code = 403
        self.mock_response.content = b"Forbidden"

        with self.assertRaises(Exception) as ctx:
            sync_config(
                client=self.mock_client,
                project_id="proj-123",
                branch_name="main",
                config_path=str(self.config_path),
            )

        self.assertIn("403", str(ctx.exception))

    def test_raises_on_graphql_errors(self) -> None:
        self.mock_response.json.return_value = {"errors": ["something went wrong"]}

        with self.assertRaises(Exception) as ctx:
            sync_config(
                client=self.mock_client,
                project_id="proj-123",
                branch_name="main",
                config_path=str(self.config_path),
            )

        self.assertEqual(ctx.exception.args[0], ["something went wrong"])


class ReadTemplatesTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.templates_dir = Path(self.temp_dir.name)

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def test_none_path_returns_empty(self) -> None:
        self.assertEqual(read_templates(None), [])

    def test_missing_directory_returns_empty(self) -> None:
        self.assertEqual(read_templates(self.templates_dir / "nonexistent"), [])

    def test_empty_directory_returns_empty(self) -> None:
        self.assertEqual(read_templates(self.templates_dir), [])

    def test_liquid_files_are_read(self) -> None:
        (self.templates_dir / "a.liquid").write_bytes(b"template a")
        (self.templates_dir / "b.liquid").write_bytes(b"template b")

        result = read_templates(self.templates_dir)

        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["name"], "a.liquid")
        self.assertEqual(
            result[0]["contents"], base64.b64encode(b"template a").decode()
        )
        self.assertEqual(result[1]["name"], "b.liquid")
        self.assertEqual(
            result[1]["contents"], base64.b64encode(b"template b").decode()
        )

    def test_non_liquid_files_are_ignored(self) -> None:
        (self.templates_dir / "template.liquid").write_bytes(b"liquid")
        (self.templates_dir / "readme.md").write_bytes(b"docs")
        (self.templates_dir / "data.yaml").write_bytes(b"yaml")

        result = read_templates(self.templates_dir)

        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["name"], "template.liquid")

    def test_accepts_string_path(self) -> None:
        (self.templates_dir / "t.liquid").write_bytes(b"content")
        result = read_templates(str(self.templates_dir))
        self.assertEqual(len(result), 1)

    def test_sorted_by_filename(self) -> None:
        for name in ["c.liquid", "a.liquid", "b.liquid"]:
            (self.templates_dir / name).write_bytes(b"x")
        result = read_templates(self.templates_dir)
        self.assertEqual(
            [r["name"] for r in result], ["a.liquid", "b.liquid", "c.liquid"]
        )


class SyncConfigWithTemplatesTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.config_path = Path(self.temp_dir.name) / "config.resim.yml"
        self.config_path.write_bytes(b"metrics_config: true\n")
        self.templates_dir = Path(self.temp_dir.name) / "templates"
        self.templates_dir.mkdir()

        self.mock_response = MagicMock()
        self.mock_response.status_code = 200
        self.mock_response.json.return_value = {"data": {"updateMetricsConfig": None}}

        self.mock_httpx = MagicMock()
        self.mock_httpx._base_url = URL("https://api.resim.ai/v1/")
        self.mock_httpx.post.return_value = self.mock_response

        self.mock_client = MagicMock()
        self.mock_client.get_httpx_client.return_value = self.mock_httpx

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def test_templates_path_read_and_passed_to_mutation(self) -> None:
        (self.templates_dir / "dashboard.liquid").write_bytes(b"hello {{ name }}")

        sync_config(
            client=self.mock_client,
            project_id="proj-123",
            branch_name="main",
            config_path=str(self.config_path),
            templates_path=self.templates_dir,
        )

        body = self.mock_httpx.post.call_args.kwargs["json"]
        template_files = body["variables"]["templateFiles"]
        self.assertEqual(len(template_files), 1)
        self.assertEqual(template_files[0]["name"], "dashboard.liquid")
        self.assertEqual(
            template_files[0]["contents"],
            base64.b64encode(b"hello {{ name }}").decode(),
        )

    def test_no_templates_path_sends_empty_list(self) -> None:
        sync_config(
            client=self.mock_client,
            project_id="proj-123",
            branch_name="main",
            config_path=str(self.config_path),
        )

        body = self.mock_httpx.post.call_args.kwargs["json"]
        self.assertEqual(body["variables"]["templateFiles"], [])


if __name__ == "__main__":
    unittest.main()
