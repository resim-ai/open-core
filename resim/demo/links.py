# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Build ReSim app URLs, so the demo can tell you exactly where to look."""

from httpx import URL

__all__ = [
    "app_base_url",
    "batch_url",
    "compare_batches_url",
    "dashboard_url",
    "dashboards_url",
]


def app_base_url(api_base_url: URL) -> str:
    """Derive the web app's origin from the customer API's base URL.

    The API and the app share a hostname prefix, so ``api.resim.ai`` becomes
    ``app.resim.ai``. Deriving it rather than hardcoding it keeps the printed
    links correct on dev and staging.
    """
    app_host = api_base_url.host.replace("api.", "app.", 1)
    return f"{api_base_url.scheme}://{app_host}"


def batch_url(app_url: str, project_id: str, batch_id: str) -> str:
    """URL of a batch's results page."""
    return f"{app_url}/projects/{project_id}/batches/{batch_id}"


def compare_batches_url(
    app_url: str, project_id: str, batch_id: str, compare_batch_id: str
) -> str:
    """URL of the A/B comparison between two batches."""
    return (
        f"{app_url}/projects/{project_id}/batches/{batch_id}"
        f"/compare-batch/batch/{compare_batch_id}"
    )


def dashboard_url(app_url: str, project_id: str, dashboard_id: str) -> str:
    """URL of a single metrics dashboard."""
    return f"{app_url}/projects/{project_id}/dashboards/{dashboard_id}"


def dashboards_url(app_url: str, project_id: str) -> str:
    """URL of a project's dashboards list.

    Used as a fallback when a dashboard's ID cannot be resolved, so the demo
    still points somewhere useful instead of printing nothing.
    """
    return f"{app_url}/projects/{project_id}/dashboards"
