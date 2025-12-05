# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import unittest
from pathlib import Path
from tempfile import NamedTemporaryFile
from unittest.mock import ANY, patch

import httpx

from resim.sdk.auth.username_password_client import UsernamePasswordClient


class UsernamePasswordClientTest(unittest.TestCase):
    """
    Tests for the UsernamePasswordClient class.
    """

    def test_username_password_client(self) -> None:
        """
        Test that the username/password token exchange works as expected.
        """
        # Given
        with (
            NamedTemporaryFile() as temp_file,
            patch("httpx.Client.get", autospec=True) as get_all_projects_mock,
            patch("httpx.post", autospec=True) as post_mock,
        ):
            post_mock.return_value = httpx.Response(
                status_code=200,
                json={
                    "access_token": "test_access_token",
                    "refresh_token": "test_refresh_token",
                    "id_token": "test_id_token",
                    "token_type": "Bearer",
                    "expires_in": 36000,
                },
            )
            get_all_projects_mock.return_value = httpx.Response(
                status_code=200,
                json={"projects": []},
            )
            # When
            client = UsernamePasswordClient(
                username="test_username",
                password="test_password",
                client_id="test_client_id",
                audience="test_audience",
                scope="test_scope",
                cache_location=Path(temp_file.name),
            )
            client.get_httpx_client().get("https://api.resim.ai/v1/projects").json()
            # Then
            post_mock.assert_called_once_with(
                "https://resim.us.auth0.com/oauth/token",
                headers={"content-type": "application/x-www-form-urlencoded"},
                data={
                    "grant_type": "http://auth0.com/oauth/grant-type/password-realm",
                    "realm": "cli-users",
                    "client_id": "test_client_id",
                    "username": "test_username",
                    "password": "test_password",
                    "audience": "test_audience",
                    "scope": "test_scope",
                },
                timeout=10.0,
            )
            # Check that the get all projects call was made with the correct headers
            get_all_projects_mock.assert_called_once_with(
                ANY, "https://api.resim.ai/v1/projects"
            )
            self.assertTrue(
                get_all_projects_mock.call_args[0][0].headers["Authorization"]
                == "Bearer test_access_token"
            )

    def test_username_password_client_cache(self) -> None:
        """
        Test that the token is not refetched if it is still valid.
        """
        # Given
        with (
            NamedTemporaryFile() as temp_file,
            patch("httpx.post", autospec=True) as post_mock,
        ):
            post_mock.return_value = httpx.Response(
                status_code=200,
                json={
                    "access_token": "test_access_token",
                    "refresh_token": "test_refresh_token",
                    "id_token": "test_id_token",
                    "token_type": "Bearer",
                    "expires_in": 36000,
                },
            )
            # When
            client = UsernamePasswordClient(
                username="test_username",
                password="test_password",
                cache_location=Path(temp_file.name),
            )
            #   get the current JWT
            first_jwt = client.token
            #   update the mock to return a new JWT
            post_mock.return_value = httpx.Response(
                status_code=200,
                json={
                    "access_token": "test_access_token_2",
                    "refresh_token": "test_refresh_token_2",
                    "id_token": "test_id_token_2",
                    "token_type": "Bearer",
                    "expires_in": 36000,
                },
            )
            # get the new JWT
            client2 = UsernamePasswordClient(
                username="test_username",
                password="test_password",
                cache_location=Path(temp_file.name),
            )
            second_jwt = client2.token
            # Then
            assert first_jwt is not None
            assert second_jwt == first_jwt
            #   reset the jwt
            client2.reset()
            # client2.reset() # coverage needed help
            client3 = UsernamePasswordClient(
                username="test_username",
                password="test_password",
                cache_location=Path(temp_file.name),
            )
            third_jwt = client3.get_jwt()
            assert third_jwt != first_jwt

    def test_username_password_client_failure(self) -> None:
        """
        Test that the username/password token exchange fails if the token is invalid.
        """
        # Given
        with (
            NamedTemporaryFile() as temp_file,
            patch("httpx.post", autospec=True) as post_mock,
            self.assertRaises(Exception) as context,
        ):
            post_mock.return_value = httpx.Response(
                status_code=401,
                json={
                    "error": "login_bad",
                    "error_description": "lorem ipsum",
                },
            )
            # When
            client = UsernamePasswordClient(
                username="test_username",
                password="test_password",
                cache_location=Path(temp_file.name),
            )
            client.get_httpx_client().get("https://api.resim.ai/v1/projects")
        # Then
        self.assertIn("Failed to get token", str(context.exception))


if __name__ == "__main__":
    unittest.main()
