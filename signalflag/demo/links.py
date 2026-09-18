# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Build SignalFlag app URLs, so the demo can tell you exactly where to look."""

from httpx import URL

__all__ = [
    "PRODUCTION_API_HOST",
    "PRODUCTION_APP_URL",
    "app_base_url",
    "batch_url",
    "compare_batches_url",
    "dashboard_url",
    "dashboards_url",
]


# The production API still answers at api.resim.ai, but the product's web app
# is SignalFlag, so links into it go to the SignalFlag domain.
PRODUCTION_API_HOST = "api.resim.ai"
PRODUCTION_APP_URL = "https://app.signalflag.ai"


def app_base_url(api_base_url: URL) -> str:
    """Work out the web app's origin from the customer API's base URL.

    Production is special-cased: the API lives at ``api.resim.ai`` while the
    app is served from ``app.signalflag.ai``, so the hostnames no longer share
    a prefix. Everywhere else (dev, staging) the API and app do share one, so
    ``api.<env>`` becomes ``app.<env>``. Deriving rather than hardcoding keeps
    the printed links correct on those deployments.
    """
    if api_base_url.host == PRODUCTION_API_HOST:
        return PRODUCTION_APP_URL
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
