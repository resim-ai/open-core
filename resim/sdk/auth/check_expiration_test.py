# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Unit test for our client token expiration logic.
"""

import unittest
from datetime import datetime, timedelta, timezone
from typing import Any
from unittest.mock import MagicMock, patch

import resim.sdk.auth.check_expiration as check_exp


class CheckExpirationTest(unittest.TestCase):
    """
    Our tests for this library.
    """

    NOW_MOCK = datetime(1918, 11, 11, hour=11, minute=11, tzinfo=timezone.utc)

    @patch("__main__.check_exp.datetime")
    def test_add_expiration_time(self, mock_now: MagicMock) -> None:
        print(f"TYPE:{type(mock_now)}")
        # SETUP
        mock_now.now.return_value = self.NOW_MOCK

        lifetime_s = 34
        token_data: dict[str, Any] = {"expires_in": lifetime_s}

        # ACTION
        check_exp.add_expiration_time(token_data=token_data)

        # VERIFICATION
        self.assertIn("expires_at", token_data)
        self.assertEqual(
            datetime.fromisoformat(token_data["expires_at"])
            - timedelta(seconds=lifetime_s),
            self.NOW_MOCK,
        )

        # ACTION / VERIFICATION
        with self.assertRaises(ValueError):
            check_exp.add_expiration_time(token_data={})

    @patch("__main__.check_exp.datetime")
    def test_is_expired(self, mock_now: MagicMock) -> None:
        # SETUP
        mock_now.now.return_value = self.NOW_MOCK
        mock_now.fromisoformat.side_effect = datetime.fromisoformat

        lifetime_s = 7200
        token_data = {"expires_in": lifetime_s}

        # ACTION / VERIFICATION
        self.assertTrue(check_exp.is_expired(token_data=token_data))

        # SETUP
        check_exp.add_expiration_time(token_data=token_data)

        # Some time passes
        ONE_HOUR_S = 3600
        mock_now.now.return_value = self.NOW_MOCK + timedelta(
            seconds=(lifetime_s - ONE_HOUR_S)
        )

        # ACTION / VERIFICATION
        self.assertFalse(check_exp.is_expired(token_data=token_data))

        # SETUP
        # Some more time passes
        ONE_HOUR_S = 3600
        mock_now.now.return_value = self.NOW_MOCK + timedelta(
            seconds=(lifetime_s - ONE_HOUR_S + 1)
        )

        # ACTION / VERIFICATION
        self.assertTrue(check_exp.is_expired(token_data=token_data))


if __name__ == "__main__":
    unittest.main()
