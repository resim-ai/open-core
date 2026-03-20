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
