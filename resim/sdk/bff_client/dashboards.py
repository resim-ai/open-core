# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Look up metrics dashboards, so callers can link straight to one in the app."""

from typing import Optional

from resim.sdk.bff_client.graphql import post
from resim.sdk.client import AuthenticatedClient

__all__ = ["find_dashboard_id"]

_LIST_DASHBOARDS = """
    query Dashboards($projectId: String!, $branchId: String, $first: Int) {
        dashboards(projectId: $projectId, branchId: $branchId, first: $first) {
            nodes {
                id
                name
            }
        }
    }
"""


def find_dashboard_id(
    client: AuthenticatedClient,
    project_id: str,
    branch_id: str,
    name: str,
) -> Optional[str]:
    """Return the ID of the dashboard called ``name`` on a branch, or ``None``.

    Dashboard names are unique per branch, so a name plus a branch identifies
    exactly one dashboard.

    Args:
        client: An authenticated ReSim API client.
        project_id: UUID of the project.
        branch_id: UUID of the branch the dashboard is scoped to.
        name: Name of the dashboard to look for.

    Raises:
        Exception: If the BFF returns a non-200 status code or GraphQL errors.
    """
    data = post(
        client,
        _LIST_DASHBOARDS,
        "Dashboards",
        {"projectId": project_id, "branchId": branch_id, "first": 100},
    )
    nodes = (data.get("dashboards") or {}).get("nodes") or []
    for node in nodes:
        if node.get("name") == name:
            return str(node["id"])
    return None
