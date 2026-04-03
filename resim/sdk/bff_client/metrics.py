import base64
from collections.abc import Sequence
from pathlib import Path
from typing import Union

import yaml
from httpx import URL

from resim.sdk.client import AuthenticatedClient
from resim.sdk.metrics.emissions import (
    merge_metrics_config_files,
    normalize_metrics_config_paths,
)


def sync_config(
    client: AuthenticatedClient,
    project_id: str,
    branch_name: str,
    config_path: Union[str, Sequence[str]] = ".resim/metrics/config.resim.yml",
    templates_path: Union[str, Path, None] = None,
) -> None:
    """
    Syncs your metrics config with ReSim.

    Args:
        client: An authenticated ReSim API client.
        project_id: The ID of the project to sync the config for.
        branch_name: The branch to associate the config with.
        config_path: Path to one metrics config file, or a sequence of paths.
            Multiple files are merged (each topic must appear in only one file);
            the combined YAML is uploaded. Defaults to
            ".resim/metrics/config.resim.yml".
        templates_path: Optional path to a directory of custom metric ``.liquid`` template
            files to sync alongside the config. If ``None`` or the directory
            does not exist, no templates are uploaded.

    Raises:
        FileNotFoundError: If a config path does not exist (when multiple paths are given).
        ValueError: If ``config_path`` is empty or topics conflict across files.
        Exception: If the server returns a non-200 status code or the
            response contains GraphQL errors.
    """
    mutation = """
        mutation UpdateMetricsConfig(
            $projectId: String!,
            $config: String!,
            $templateFiles: [MetricsTemplate!]!,
            $branch: String) {
              updateMetricsConfig(
                projectId: $projectId,
                config: $config,
                templateFiles: $templateFiles,
                branch: $branch
            )
        }
    """
    paths = normalize_metrics_config_paths(config_path)
    if not paths:
        raise ValueError("config_path must not be empty")
    if len(paths) == 1:
        config_bytes = paths[0].read_bytes()
    else:
        merged = merge_metrics_config_files(paths)
        config_bytes = yaml.safe_dump(
            merged, sort_keys=False, allow_unicode=True
        ).encode("utf-8")

    body = {
        "query": mutation,
        "operationName": "UpdateMetricsConfig",
        "variables": {
            "projectId": project_id,
            "config": base64.b64encode(config_bytes).decode(),
            "templateFiles": read_templates(templates_path),
            "branch": branch_name,
        },
    }
    httpx_client = client.get_httpx_client()
    bff_url = _get_bff_url(httpx_client._base_url)

    response = httpx_client.post(bff_url, json=body)
    if response.status_code != 200:
        raise Exception(
            f"failed to sync metrics config {response.status_code}: {response.content}"
        )
    if "errors" in response.json():
        raise Exception(response.json()["errors"])


def read_templates(path: Union[str, Path, None]) -> list[dict[str, str]]:
    """Read Liquid template files from a directory for use with sync_config.

    Scans the given directory for ``*.liquid`` files and returns them as a list
    of dicts with ``name`` (filename) and ``contents`` (base64-encoded bytes),
    matching the format expected by the ``UpdateMetricsConfig`` mutation.

    Args:
        path: Directory containing ``.liquid`` template files. If ``None`` or
            the directory does not exist, an empty list is returned.

    Returns:
        A list of ``{"name": str, "contents": str}`` dicts, one per ``.liquid``
        file found, sorted by filename for deterministic ordering.
    """
    if path is None:
        return []
    directory = Path(path)
    if not directory.is_dir():
        return []
    templates = []
    for entry in sorted(directory.iterdir()):
        if entry.is_file() and entry.suffix.lower() == ".liquid":
            templates.append(
                {
                    "name": entry.name,
                    "contents": base64.b64encode(entry.read_bytes()).decode(),
                }
            )
    return templates


def _get_bff_url(api_base_url: URL) -> str:
    bff_host = api_base_url.host.replace("api.", "bff.", 1)
    return str(api_base_url.copy_with(host=bff_host, raw_path=b"/graphql"))
