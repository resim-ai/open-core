# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Unit tests for OrientedBox pybinding
"""

import unittest

import numpy as np

import resim.geometry.python.oriented_box_python as oribox
import resim.transforms.python.se3_python as se3


class OrientedBoxPythonTest(unittest.TestCase):
    """Unit tests for Oriented Box pybinding"""

    def oriented_box_test(self) -> None:
        se3 = se3.SE3.identity()


if __name__ == "__main__":
    unittest.main()
