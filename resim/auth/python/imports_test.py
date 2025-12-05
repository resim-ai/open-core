# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Test that the moved imports work as expected.
"""

import unittest

from resim.auth.python.check_expiration import (
    add_expiration_time as add_expiration_time_moved,
    is_expired as is_expired_moved,
)
from resim.auth.python.device_code_client import (
    DeviceCodeClient as DeviceCodeClientMoved,
)
from resim.auth.python.username_password_client import (
    UsernamePasswordClient as UsernamePasswordClientMoved,
)
from resim.auth.python.const import DEFAULT_BASE_URL as DEFAULT_BASE_URL_MOVED

from resim.sdk.auth.check_expiration import add_expiration_time, is_expired
from resim.sdk.auth.device_code_client import DeviceCodeClient
from resim.sdk.auth.username_password_client import UsernamePasswordClient
from resim.sdk.auth.const import DEFAULT_BASE_URL


class ImportsTest(unittest.TestCase):
    """
    Test that the moved imports work as expected.
    """

    def test_imports(self) -> None:
        self.assertEqual(add_expiration_time_moved, add_expiration_time)
        self.assertEqual(is_expired_moved, is_expired)
        self.assertEqual(DeviceCodeClientMoved, DeviceCodeClient)
        self.assertEqual(UsernamePasswordClientMoved, UsernamePasswordClient)
        self.assertEqual(DEFAULT_BASE_URL_MOVED, DEFAULT_BASE_URL)


if __name__ == "__main__":
    unittest.main()
