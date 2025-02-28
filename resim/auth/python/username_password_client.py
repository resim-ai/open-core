# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
This file contains a Client class used to communicate with an OAuth
2 authentication server to procure a JSON Web Token (jwt) for use with
ReSim's API (https://api.resim.ai) given a username and password.
"""

import json
import os
import pathlib
from typing import Any, Optional

import httpx
from resim_python_client import AuthenticatedClient

import resim.auth.python.check_expiration as check_exp
from resim.auth.python.const import (
    DEFAULT_AUDIENCE,
    DEFAULT_BASE_URL,
    DEFAULT_CACHE_LOCATION,
    DEFAULT_DOMAIN,
    DEFAULT_SCOPE,
)

PASSWORD_AUTH_CLIENT_ID = "0Ip56H1LLAo6Dc6IfePaNzgpUxbJGyVI"


class UsernamePasswordClient(AuthenticatedClient):
    """
    The client class which manages the token as well as other
    configuration data for authentication.
    """

    def __init__(
        self,
        *,
        base_url: str = DEFAULT_BASE_URL,
        domain: str = DEFAULT_DOMAIN,
        client_id: str = PASSWORD_AUTH_CLIENT_ID,
        scope: str = DEFAULT_SCOPE,
        audience: str = DEFAULT_AUDIENCE,
        cache_location: pathlib.Path = DEFAULT_CACHE_LOCATION,
        username: Optional[str] = os.getenv("RESIM_USERNAME"),
        password: Optional[str] = os.getenv("RESIM_PASSWORD"),
        **kwargs: Any,
    ):
        self.token: Optional[dict[str, Any]] = None

        self._client_id = client_id
        self._cache_location = cache_location
        self._domain = domain
        self._scope = scope
        self._audience = audience

        assert username is not None, "Username is required"
        assert password is not None, "Password is required"
        self._username = username
        self._password = password

        super().__init__(
            token=self.get_jwt()["access_token"],
            base_url=base_url,
            **kwargs,
        )

    def reset(self) -> None:
        """Clear the local token cache and the internal token."""
        self.token = None
        if self._cache_location.exists():
            self._cache_location.unlink()

    def get_jwt(self) -> dict[str, Any]:
        """Get the current token, fetching if necessary."""
        if self.token is None and self._cache_location.exists():
            assert self._cache_location.is_file(), (
                "Directory detected in cache location!"
            )
            try:
                with open(self._cache_location, "r", encoding="utf-8") as cache:
                    self.token = json.load(cache)
            except json.JSONDecodeError as _:
                self.token = None

        if self.token is None or check_exp.is_expired(token_data=self.token):
            self.token = _get_new_token(
                domain=self._domain,
                client_id=self._client_id,
                scope=self._scope,
                audience=self._audience,
                username=self._username,
                password=self._password,
            )
            self._cache_location.parent.mkdir(parents=True, exist_ok=True)
            with open(self._cache_location, "w", encoding="utf-8") as cache:
                cache.write(json.dumps(self.token, indent=4))
        return self.token


def _get_new_token(
    *,
    domain: str,
    client_id: str,
    scope: str,
    audience: str,
    username: str,
    password: str,
) -> dict[str, Any]:
    token_url = domain + "/oauth/token"

    payload = {
        "grant_type": "http://auth0.com/oauth/grant-type/password-realm",
        "realm": "cli-users",
        "client_id": client_id,
        "username": username,
        "password": password,
        "audience": audience,
        "scope": scope,
    }

    resp = httpx.post(
        token_url,
        headers={"content-type": "application/x-www-form-urlencoded"},
        data=payload,
        timeout=10.0,
    )
    assert resp.status_code == 200, f"Failed to get token: {resp.text}"
    token: dict[str, Any] = resp.json()

    check_exp.add_expiration_time(token_data=token)

    return token
