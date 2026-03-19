import base64
from httpx import URL
from resim.sdk.client import AuthenticatedClient


def sync_config(
    client: AuthenticatedClient,
    project_id: str,
    branch_name: str,
    config_path: str = ".resim/metrics/config.resim.yml",
    templates: list[dict[str, str]] = [],
) -> None:
    """Syncs your metrics config with ReSim."""
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
    body = {
        "query": mutation,
        "operationName": "UpdateMetricsConfig",
        "variables": {
            "projectId": project_id,
            "config": base64.b64encode(open(config_path, mode="rb").read()).decode(),
            "templateFiles": templates,
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


def _get_bff_url(api_base_url: URL) -> str:
    bff_host = api_base_url.host.replace("api.", "bff.", 1)
    return str(api_base_url.copy_with(host=bff_host, raw_path=b"/graphql"))
