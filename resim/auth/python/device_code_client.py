# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
This file contains a Client class used to communicate with an OAuth
2 authentication server to procure a JSON Web Token (jwt) for use with
ReSim's API (https://api.resim.ai) via the device code flow.
"""

import json
import pathlib
import typing
from http import HTTPStatus

import polling2
import requests

import resim.auth.python.check_expiration as check_exp
from resim.auth.python.const import (
    DEFAULT_AUDIENCE,
    DEFAULT_CACHE_LOCATION,
    DEFAULT_DOMAIN,
    DEFAULT_SCOPE,
)

DEVICE_CODE_CLIENT_ID = "gTp1Y0kOyQ7QzIo2lZm0auGM6FJZZVvy"


class DeviceCodeClient:
    """
    The client class which manages the token as well as other
    configuration data for authentication.
    """

    def __init__(
        self,
        *,
        domain: str = DEFAULT_DOMAIN,
        client_id: str = DEVICE_CODE_CLIENT_ID,
        scope: str = DEFAULT_SCOPE,
        audience: str = DEFAULT_AUDIENCE,
        cache_location: pathlib.Path = DEFAULT_CACHE_LOCATION,
    ):
        self._token: typing.Optional[dict[str, typing.Any]] = None
        self._client_id = client_id
        self._cache_location = cache_location
        self._domain = domain
        self._scope = scope
        self._audience = audience

    def refresh(self) -> None:
        """Clear the local token cache and the internal token."""
        self._token = None
        if self._cache_location.exists():
            self._cache_location.unlink()

    def get_jwt(self) -> dict[str, typing.Any]:
        """Get the current token, fetching if necessary."""
        if self._token is None and self._cache_location.exists():
            assert self._cache_location.is_file(), (
                "Directory detected in cache location!"
            )
            with open(self._cache_location, "r", encoding="utf-8") as cache:
                self._token = json.load(cache)

        if self._token is None or check_exp.is_expired(token_data=self._token):
            self._token = _get_new_token(
                domain=self._domain,
                client_id=self._client_id,
                scope=self._scope,
                audience=self._audience,
            )
            self._cache_location.parent.mkdir(parents=True, exist_ok=True)
            with open(self._cache_location, "w", encoding="utf-8") as cache:
                cache.write(json.dumps(self._token, indent=4))
        return self._token


def _get_new_token(
    *, domain: str, client_id: str, scope: str, audience: str
) -> dict[str, typing.Any]:
    payload = {
        "client_id": client_id,
        "scope": scope,
        "audience": audience,
    }

    device_code_response = requests.post(domain + "/oauth/device/code", data=payload)

    if device_code_response.status_code != HTTPStatus.OK:
        raise RuntimeError("Failed to fetch device code!")

    device_code_data = device_code_response.json()

    message = f"""Authenticating by Device Code

Please navigate to: {device_code_data["verification_uri_complete"]}
"""
    print(message)

    device_code = device_code_data["device_code"]
    polling_interval = device_code_data["interval"]
    timeout = device_code_data["expires_in"]

    payload = {
        "client_id": client_id,
        "device_code": device_code,
        "grant_type": "urn:ietf:params:oauth:grant-type:device_code",
    }

    def poll_once() -> typing.Optional[requests.Response]:
        token_response = requests.post(domain + "/oauth/token", data=payload)
        return token_response if token_response.status_code == HTTPStatus.OK else None

    token_response = polling2.poll(poll_once, step=polling_interval, timeout=timeout)

    token_data: dict[str, typing.Any] = token_response.json()

    check_exp.add_expiration_time(token_data=token_data)

    return token_data
