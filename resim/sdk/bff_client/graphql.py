# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Shared plumbing for talking to the ReSim BFF's GraphQL endpoint."""

from typing import Any

from httpx import URL

from resim.sdk.client import AuthenticatedClient

__all__ = ["bff_url", "post"]


def bff_url(api_base_url: URL) -> str:
    """Derive the BFF GraphQL endpoint from the customer API's base URL.

    The two services share a hostname prefix, so ``api.resim.ai`` becomes
    ``bff.resim.ai``. Deriving it this way keeps dev and staging working without
    any extra configuration.
    """
    bff_host = api_base_url.host.replace("api.", "bff.", 1)
    return str(api_base_url.copy_with(host=bff_host, raw_path=b"/graphql"))


def post(
    client: AuthenticatedClient,
    query: str,
    operation_name: str,
    variables: dict[str, Any],
) -> dict[str, Any]:
    """Run a GraphQL operation against the BFF and return its ``data`` payload.

    Args:
        client: An authenticated ReSim API client.
        query: The GraphQL document to execute.
        operation_name: Name of the operation within ``query``.
        variables: Variables for the operation.

    Raises:
        Exception: If the server returns a non-200 status code or the response
            contains GraphQL errors.
    """
    httpx_client = client.get_httpx_client()
    response = httpx_client.post(
        bff_url(httpx_client._base_url),
        json={
            "query": query,
            "operationName": operation_name,
            "variables": variables,
        },
    )
    if response.status_code != 200:
        raise Exception(
            f"failed to run {operation_name} {response.status_code}: "
            f"{response.content!r}"
        )
    payload = response.json()
    if "errors" in payload:
        raise Exception(payload["errors"])
    data = payload.get("data")
    return data if isinstance(data, dict) else {}
