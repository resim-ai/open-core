# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Unit tests for Polygon Distance pybinding
"""

import unittest

import numpy as np

import resim.geometry.python.box_box_distance_python as bbd
import resim.geometry.python.oriented_box_python as ob
import resim.transforms.python.se3_python as se3


class BoxBoxDistancePythonTest(unittest.TestCase):
    """Unit tests for box box distance pybinding.

    This is an extermely simple test since the full functionality is tested in the C++ tests.
    """

    def test_box_box_distance(self) -> None:
        # SETUP
        box_a_translation = np.zeros(3)
        box_b_translation = np.array([3.0, 0.0, 0.0])
        extents = 2.0 * np.ones(3)
        box_a = ob.OrientedBox(
            reference_from_box=se3.SE3(box_a_translation),
            extents=extents,
        )
        box_b = ob.OrientedBox(
            reference_from_box=se3.SE3(box_b_translation),
            extents=extents,
        )

        # ACTION
        distance = bbd.box_box_distance(box_a, box_b)

        # VERIFICATION
        self.assertAlmostEqual(
            distance, (box_b_translation - box_a_translation)[0] - extents[0]
        )


if __name__ == "__main__":
    unittest.main()
