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

from resim.sdk.bff_client.metrics import sync_config


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
        self.assertEqual(variables["templateFiles"], [])
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


if __name__ == "__main__":
    unittest.main()
