# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import json
import pathlib
import polling2
import requests

from http import HTTPStatus


CACHE_LOCATION = pathlib.Path.home() / ".resim" / "token.json"


class DeviceCodeClient:
    def __init__(self, *, domain: str, client_id: str,
                 cache_location: pathlib.Path = CACHE_LOCATION):

        self._token = None
        self._client_id = client_id
        self._cache_location = cache_location
        self._domain = domain

    def refresh(self) -> None:
        if self._cache_location.exists():
            self._cache_location.unlink()
        self._token = None

    def get_jwt(self) -> dict[str, any]:
        if self._token is None and self._cache_location.exists():
            assert self._cache_location.is_file(), "Directory detected in cache location!"
            with open(self._cache_location, "r") as cache:
                self._token = json.load(cache)
        elif self._token is None:
            self._token = _get_new_token(
                domain=self._domain, client_id=self._client_id)
            with open(self._cache_location, "w") as cache:
                cache.write(json.dumps(self._token, indent=4))
        return self._token


def _get_new_token(*, domain: str, client_id: str):
    payload = {
        "client_id": client_id,
        "scope": "offline_access",
        "audience": "https://api.resim.ai",
    }

    device_code_response = requests.post(
        domain + "/oauth/device/code", data=payload)

    if device_code_response.status_code != HTTPStatus.OK:
        raise RuntimeError("Failed to fetch device code!")

    device_code_data = device_code_response.json()
    print(device_code_data)

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

    def poll_once():
        token_response = requests.post(domain + "/oauth/token", data=payload)
        print(token_response.status_code)
        print(token_response.json())
        return token_response if token_response.status_code == HTTPStatus.OK else None

    token_response = polling2.poll(
        poll_once,
        step=polling_interval,
        timeout=timeout)

    token_data = token_response.json()

    return token_data
