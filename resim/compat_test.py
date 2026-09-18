# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Checks that the legacy resim.sdk / resim.demo paths alias the new packages."""

import importlib
import unittest
from unittest import mock


class CompatAliasTest(unittest.TestCase):
    def test_top_level_packages_are_the_same_objects(self) -> None:
        import resim.demo
        import resim.sdk
        import signalflag.demo
        import signalflag.sdk

        self.assertIs(resim.sdk, signalflag.sdk)
        self.assertIs(resim.demo, signalflag.demo)

    def test_submodules_are_the_same_objects(self) -> None:
        import resim.sdk.auth.const as old_const
        import resim.sdk.client.models as old_models
        import signalflag.sdk.auth.const as new_const
        import signalflag.sdk.client.models as new_models

        self.assertIs(old_const, new_const)
        self.assertIs(old_models, new_models)
        self.assertEqual(new_const.__name__, "signalflag.sdk.auth.const")

    def test_from_imports_resolve(self) -> None:
        from resim.demo import run as old_run
        from resim.sdk.batch import Batch as OldBatch
        from signalflag.demo import run as new_run
        from signalflag.sdk.batch import Batch as NewBatch

        self.assertIs(old_run, new_run)
        self.assertIs(OldBatch, NewBatch)

    def test_patching_old_path_affects_new_module(self) -> None:
        import signalflag.sdk.test as new_test

        sentinel = object()
        with mock.patch("resim.sdk.test.httpx", sentinel):
            self.assertIs(new_test.httpx, sentinel)

    def test_missing_submodule_still_raises(self) -> None:
        with self.assertRaises(ModuleNotFoundError):
            importlib.import_module("resim.sdk.does_not_exist")


if __name__ == "__main__":
    unittest.main()
