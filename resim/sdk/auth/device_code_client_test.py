# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Unit test for our python device code client.
"""

import pathlib
import random
import string
import tempfile
import types
import typing
import unittest
import unittest.mock
from http import HTTPStatus

import resim.sdk.auth.device_code_client as dcc

DOMAIN = "https://resim.us.auth0.com"
CLIENT_ID = "U6EHPrmAIJf8QUUzldGa4cN26XP58I2a"
TOKEN = (
    "h8aNbLY2kkR1Pvr6q59ap6hFW5IU18IXm4mASp3QMc6dd3QeZJNqjqyMBy3aWuWCTFiav3T0wRBUTtmG"
    + "ToJJkhFpoU3fjaJ7HEBKNZwvA8HtF8noqaBO5oLE854fsL8p2r9xkxwG6pvaoVQAx0lQ8ubO94qty2CI"
    + "huLXkZyt6riZhdo6w8Mkxi0L36UBDK02TC5uZlbDtMRURzkessDYxZhrIwHDEJX5aURCvkyznJmSHJlJ"
    + "IvN3W2vGr6ljYIMw"
)
REFRESH_TOKEN = (
    "qaBO5oLE854fsL8p2r9xkxwG6pvaoVQAx0lQ8ubO94qty2CIhuLXkZyt6riZhdo6w8Mkxi0L36UBDK02"
    + "TC5uZl"
)


class MockServer:
    """A mock server to be patched into our DeviceCodeClient unit test below"""

    def __init__(self, *, testcase: unittest.TestCase, expires_in: int = 7200):
        self._device_code = self._generate_device_code()
        self._user_code = self._generate_user_code()
        self._authenticated = False
        self._testcase = testcase
        self._scope = ""
        self._expires_in = expires_in

        self.num_device_code_requests = 0
        self.num_token_requests = 0

    def handle_post(
        self, uri: str, *, data: dict[str, typing.Any]
    ) -> types.SimpleNamespace:
        """Handle an incoming post to the mock server and delegate based on endpoint."""
        if uri.startswith(DOMAIN):
            endpoint = uri[len(DOMAIN) :]
            if endpoint == "/oauth/device/code":
                return self.handle_device_code(data=data)
            if endpoint == "/oauth/token":
                return self.handle_token(data=data)
        result = types.SimpleNamespace()
        result.status_code = HTTPStatus.NOT_FOUND
        return result

    def _generate_device_code(self) -> str:
        """Generate a random device code."""
        return "".join(random.choices(string.ascii_letters, k=24))

    def _generate_user_code(self) -> str:
        """Generate a random user code."""
        return "".join(
            random.choices(string.ascii_uppercase, k=4)
            + ["-"]
            + random.choices(string.ascii_uppercase, k=4)
        )

    def handle_device_code(
        self, *, data: dict[str, typing.Any]
    ) -> types.SimpleNamespace:
        """Handle a device code post and test its contents."""
        self.num_device_code_requests += 1
        self._testcase.assertIn("client_id", data)
        self._testcase.assertIn("scope", data)
        self._testcase.assertIn("audience", data)
        self._testcase.assertEqual(data["client_id"], CLIENT_ID)
        self._testcase.assertEqual(data["scope"], "offline_access")
        self._testcase.assertEqual(data["audience"], "https://api.resim.ai")

        self._scope = data["scope"]
        result = types.SimpleNamespace()
        result.status_code = HTTPStatus.OK
        verification_uri = DOMAIN + "/activate"
        result.json = lambda: {
            "device_code": self._device_code,
            "user_code": self._user_code,
            "verification_uri": verification_uri,
            "expires_in": 900,
            "interval": 0,  # Since this is a unit test
            "verification_uri_complete": f"{verification_uri}?user_code={self._user_code}",
        }
        return result

    def handle_token(self, *, data: dict[str, typing.Any]) -> types.SimpleNamespace:
        """Handle a token post and test its contents."""
        self._testcase.assertIn("client_id", data)
        self._testcase.assertIn("device_code", data)
        self._testcase.assertIn("grant_type", data)
        self._testcase.assertEqual(data["client_id"], CLIENT_ID)
        self._testcase.assertEqual(data["device_code"], self._device_code)
        self._testcase.assertEqual(
            data["grant_type"], "urn:ietf:params:oauth:grant-type:device_code"
        )

        self.num_token_requests += 1
        result = types.SimpleNamespace()
        if not self._authenticated:
            result.status_code = HTTPStatus.FORBIDDEN
            result.json = lambda: {
                "error": "authorization_pending",
                "error_description": "User has yet to authorize device code.",
            }
            self._authenticated = True
        else:
            result.status_code = HTTPStatus.OK
            result.json = lambda: {
                "access_token": TOKEN,
                "refresh_token": REFRESH_TOKEN,
                "scope": self._scope,
                "expires_in": self._expires_in,
                "token_type": "Bearer",
            }

        return result


class DeviceCodeClientTest(unittest.TestCase):
    """
    Our actual unit test case.
    """

    def test_device_code_client(self) -> None:
        """
        Test that we can get the expected token with the expected queries to the server.
        """
        with (
            unittest.mock.patch("requests.post") as mock,
            tempfile.TemporaryDirectory() as tmpdir,
        ):

            def side_effect(
                uri: str, *, data: dict[str, typing.Any]
            ) -> types.SimpleNamespace:
                return server.handle_post(uri, data=data)

            mock.side_effect = side_effect

            # Test getting the token
            server = MockServer(testcase=self)
            client = dcc.DeviceCodeClient(
                domain=DOMAIN,
                client_id=CLIENT_ID,
                cache_location=pathlib.Path(tmpdir) / "token.json",
            )
            token = client.get_jwt()
            self.assertEqual(token["access_token"], TOKEN)
            self.assertEqual(server.num_device_code_requests, 1)
            self.assertEqual(server.num_token_requests, 2)

            # Test getting it again (caching behavior)
            server = MockServer(testcase=self)
            client = dcc.DeviceCodeClient(
                domain=DOMAIN,
                client_id=CLIENT_ID,
                cache_location=pathlib.Path(tmpdir) / "token.json",
            )
            token = client.get_jwt()
            self.assertEqual(token["access_token"], TOKEN)
            self.assertEqual(server.num_device_code_requests, 0)
            self.assertEqual(server.num_token_requests, 0)

            # Test refreshing
            server = MockServer(testcase=self)
            client.refresh()
            token = client.get_jwt()
            self.assertEqual(token, client.get_jwt())
            self.assertEqual(token["access_token"], TOKEN)
            self.assertEqual(server.num_device_code_requests, 1)
            self.assertEqual(server.num_token_requests, 2)

            # Test expiration
            # We expect two separate requests for two separate
            # "get_jwt()" calls because the token expires
            # instantly due to the one hour buffer.
            server = MockServer(testcase=self, expires_in=3599)
            client.refresh()
            token = client.get_jwt()
            self.assertEqual(token["access_token"], TOKEN)
            self.assertEqual(client.get_jwt()["access_token"], TOKEN)
            self.assertEqual(server.num_device_code_requests, 2)

    def test_fail_on_404(self) -> None:
        """
        Verify that we raise on 404.
        """
        with (
            unittest.mock.patch("requests.post") as mock,
            tempfile.TemporaryDirectory() as tmpdir,
        ):

            def side_effect(
                uri: str, *, data: dict[str, typing.Any]
            ) -> types.SimpleNamespace:
                return server.handle_post(uri, data=data)

            mock.side_effect = side_effect

            server = MockServer(testcase=self)
            client = dcc.DeviceCodeClient(
                domain="www.mybaddomain.com",
                client_id=CLIENT_ID,
                cache_location=pathlib.Path(tmpdir) / "token.json",
            )
            with self.assertRaises(RuntimeError):
                client.get_jwt()


if __name__ == "__main__":
    unittest.main()
