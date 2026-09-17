# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import pathlib
import tempfile
import unittest

from signalflag.sdk.auth import const


class ConfigDirTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.home = pathlib.Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)

    def test_uses_signalflag_dir_when_it_exists(self) -> None:
        (self.home / ".signalflag").mkdir()
        (self.home / ".resim").mkdir()
        self.assertEqual(const.config_dir(self.home), self.home / ".signalflag")

    def test_falls_back_to_legacy_dir_when_only_it_exists(self) -> None:
        (self.home / ".resim").mkdir()
        self.assertEqual(const.config_dir(self.home), self.home / ".resim")

    def test_uses_signalflag_dir_when_neither_exists(self) -> None:
        self.assertEqual(const.config_dir(self.home), self.home / ".signalflag")
        # Resolution has no side effects; the clients create it on first write.
        self.assertFalse((self.home / ".signalflag").exists())

    def test_legacy_must_be_a_directory_to_count(self) -> None:
        (self.home / ".resim").write_text("not a directory")
        self.assertEqual(const.config_dir(self.home), self.home / ".signalflag")

    def test_default_cache_location(self) -> None:
        (self.home / ".resim").mkdir()
        self.assertEqual(
            const.default_cache_location(self.home), self.home / ".resim" / "token.json"
        )
        self.assertEqual(const.DEFAULT_CACHE_LOCATION.name, "token.json")


if __name__ == "__main__":
    unittest.main()
