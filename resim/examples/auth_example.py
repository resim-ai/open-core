# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Example script demonstrating both authentication methods for ReSim API.

This script validates that both DeviceCodeClient and UsernamePasswordClient
can successfully authenticate and communicate with the ReSim API.

Usage:
    # Using device code flow (interactive)
    bazel run //resim/examples:auth_example

    # Using username/password (requires environment variables)
    RESIM_USERNAME=your_username RESIM_PASSWORD=your_password bazel run //resim/examples:auth_example
"""

import os
import sys
from typing import Callable
from tempfile import NamedTemporaryFile
import traceback
from pathlib import Path
from resim.sdk.auth.device_code_client import DeviceCodeClient
from resim.sdk.auth.username_password_client import UsernamePasswordClient
from resim.sdk.client.api.projects import list_projects
from resim.sdk.client import AuthenticatedClient


def _test_auth_and_api(
    client_factory: Callable[[Path], AuthenticatedClient],
    test_name: str,
    auth_message: str = "Authenticating...",
    show_traceback: bool = False,
) -> bool:
    """
    Common function to test authentication and API communication.

    Args:
        client_factory: Function that creates a client given a cache location path
        test_name: Name of the authentication method being tested
        auth_message: Message to display during authentication
        show_traceback: Whether to show full traceback on error

    Returns:
        True if authentication and API call succeeded, False otherwise.
    """
    print("\n" + "=" * 60)
    print(f"Testing {test_name}")
    print("=" * 60)

    try:
        with NamedTemporaryFile() as temp_file:
            cache_path = Path(temp_file.name)
            client = client_factory(cache_path)

            print("Testing API communication by listing projects...")
            with client as api_client:
                print(auth_message)
                # The client already authenticated in __init__, but we can verify
                jwt = client.get_jwt()
                assert jwt is not None, "Failed to get JWT token"
                assert "access_token" in jwt, "JWT token missing access_token"
                print(
                    f"✓ Successfully authenticated (token expires in {jwt.get('expires_in', 'unknown')} seconds)"
                )

                projects_output = list_projects.sync(client=api_client)
                if projects_output is None:
                    print("✗ Failed to list projects")
                    return False

                print("✓ Successfully connected to ReSim API")
                print(f"  Found {len(projects_output.projects)} project(s)")
                if projects_output.projects:
                    print("  Project names:")
                    for project in projects_output.projects[:5]:  # Show first 5
                        print(f"    - {project.name} (ID: {project.project_id})")
                    if len(projects_output.projects) > 5:
                        print(f"    ... and {len(projects_output.projects) - 5} more")

            return True

    except Exception as e:
        print(f"✗ {test_name} failed: {e}")
        if show_traceback:
            print(traceback.format_exc())
        return False


def test_device_code_auth() -> bool:
    """
    Test authentication using DeviceCodeClient.

    Returns:
        True if authentication and API call succeeded, False otherwise.
    """
    return _test_auth_and_api(
        client_factory=lambda cache_path: DeviceCodeClient(cache_location=cache_path),
        test_name="Device Code Authentication",
        auth_message="Authenticating (this may prompt you to visit a URL)...",
    )


def test_username_password_auth() -> bool:
    """
    Test authentication using UsernamePasswordClient.

    Returns:
        True if authentication and API call succeeded, False otherwise.
    """
    username = os.getenv("RESIM_USERNAME")
    password = os.getenv("RESIM_PASSWORD")

    if not username or not password:
        print("\n" + "=" * 60)
        print("Testing Username/Password Authentication")
        print("=" * 60)
        print(
            "⚠ Skipping username/password test (RESIM_USERNAME and RESIM_PASSWORD not set)"
        )
        return True  # Not a failure, just skipped

    return _test_auth_and_api(
        client_factory=lambda cache_path: UsernamePasswordClient(
            username=username,
            password=password,
            cache_location=cache_path,
        ),
        test_name="Username/Password Authentication",
        show_traceback=True,
    )


def main() -> None:
    """
    Main function to test both authentication methods.
    """
    print("ReSim Authentication Example")
    print("=" * 60)
    print(
        "This script validates both authentication methods can communicate with ReSim API"
    )
    print()

    device_code_success = test_device_code_auth()
    username_password_success = test_username_password_auth()

    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    print(f"Device Code Auth:     {'✓ PASSED' if device_code_success else '✗ FAILED'}")
    print(
        f"Username/Password Auth: {'✓ PASSED' if username_password_success else '✗ FAILED'}"
    )

    # Exit with error if device code failed (it should always work)
    # Username/password is optional (may be skipped if env vars not set)
    if not device_code_success:
        print("\n✗ Device code authentication is required and failed!")
        sys.exit(1)

    print("\n✓ All authentication methods validated successfully!")
    sys.exit(0)


if __name__ == "__main__":
    main()
