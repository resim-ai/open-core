# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# Copied from https://bazel.build/tutorials/cc-toolchain-config

"""
A simple test that our python distribution is set up correctly.
"""

import sys
import unittest


class TestPythonVersion(unittest.TestCase):
    """
    A simple test that our python distribution is set up correctly.
    """
    def test_python_version(self) -> None:
        """
        Test that the python version is what we expect.
        """
        version_info = sys.version_info
        self.assertEqual(version_info[0], 3)
        self.assertEqual(version_info[1], 10)
        self.assertEqual(version_info[2], 13)


if __name__ == '__main__':
    unittest.main()
