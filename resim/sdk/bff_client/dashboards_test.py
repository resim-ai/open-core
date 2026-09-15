# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import unittest
from unittest.mock import MagicMock

from httpx import URL

from resim.sdk.bff_client.dashboards import find_dashboard_id


def _nodes(*names: str) -> dict:
    return {
        "data": {
            "dashboards": {
                "nodes": [{"id": f"id-{name}", "name": name} for name in names]
            }
        }
    }


class FindDashboardIdTest(unittest.TestCase):
    def setUp(self) -> None:
        self.mock_response = MagicMock()
        self.mock_response.status_code = 200
        self.mock_response.json.return_value = _nodes("Other", "SDK Demo Trends")

        self.mock_httpx = MagicMock()
        self.mock_httpx._base_url = URL("https://api.resim.ai/v1/")
        self.mock_httpx.post.return_value = self.mock_response

        self.mock_client = MagicMock()
        self.mock_client.get_httpx_client.return_value = self.mock_httpx

    def _find(self, name: str = "SDK Demo Trends") -> object:
        return find_dashboard_id(self.mock_client, "proj-123", "branch-456", name)

    def test_returns_matching_dashboard_id(self) -> None:
        self.assertEqual(self._find(), "id-SDK Demo Trends")

    def test_posts_to_bff_host_with_project_and_branch(self) -> None:
        self._find()

        call = self.mock_httpx.post.call_args
        self.assertIn("bff.resim.ai", call.args[0])
        variables = call.kwargs["json"]["variables"]
        self.assertEqual(variables["projectId"], "proj-123")
        self.assertEqual(variables["branchId"], "branch-456")

    def test_returns_none_when_name_absent(self) -> None:
        self.assertIsNone(self._find("Nonexistent"))

    def test_returns_none_when_no_dashboards(self) -> None:
        self.mock_response.json.return_value = {"data": {"dashboards": {"nodes": []}}}
        self.assertIsNone(self._find())

    def test_raises_on_non_200_status(self) -> None:
        self.mock_response.status_code = 500
        self.mock_response.content = b"boom"

        with self.assertRaises(Exception) as ctx:
            self._find()

        self.assertIn("500", str(ctx.exception))

    def test_raises_on_graphql_errors(self) -> None:
        self.mock_response.json.return_value = {"errors": ["nope"]}

        with self.assertRaises(Exception) as ctx:
            self._find()

        self.assertEqual(ctx.exception.args[0], ["nope"])


if __name__ == "__main__":
    unittest.main()
