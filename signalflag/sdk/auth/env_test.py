# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os
import unittest
from unittest.mock import patch

from signalflag.sdk.auth import env


class GetenvTest(unittest.TestCase):
    def test_prefers_the_signalflag_name(self) -> None:
        with patch.dict(
            os.environ, {"SIGNALFLAG_USERNAME": "new", "RESIM_USERNAME": "old"}
        ):
            self.assertEqual(env.getenv(env.USERNAME), "new")

    def test_falls_back_to_the_legacy_name(self) -> None:
        with patch.dict(os.environ, {"RESIM_USERNAME": "old"}, clear=True):
            self.assertEqual(env.getenv(env.USERNAME), "old")

    def test_an_empty_new_name_does_not_hide_the_legacy_value(self) -> None:
        with patch.dict(
            os.environ, {"SIGNALFLAG_USERNAME": "", "RESIM_USERNAME": "old"}
        ):
            self.assertEqual(env.getenv(env.USERNAME), "old")

    def test_default_when_neither_is_set(self) -> None:
        with patch.dict(os.environ, {}, clear=True):
            self.assertIsNone(env.getenv(env.USERNAME))
            self.assertEqual(env.getenv(env.USERNAME, "x"), "x")

    def test_legacy_name(self) -> None:
        self.assertEqual(env.legacy_name("SIGNALFLAG_API_URL"), "RESIM_API_URL")
        with self.assertRaises(ValueError):
            env.legacy_name("RESIM_API_URL")

    def test_describe_names_both(self) -> None:
        self.assertEqual(
            env.describe(env.PASSWORD), "SIGNALFLAG_PASSWORD (or RESIM_PASSWORD)"
        )


if __name__ == "__main__":
    unittest.main()
