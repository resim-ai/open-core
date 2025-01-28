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

    def test_constructor(self) -> None:
        # SETUP
        DIM = 3
        reference_from_box = se3.SE3.exp(np.random.rand(se3.SE3.DOF))
        extents = np.random.rand(DIM)

        # ACTION
        box = oribox.OrientedBox(reference_from_box=reference_from_box, extents=extents)

        # VERIFICATION
        self.assertTrue(reference_from_box.is_approx(box.reference_from_box()))
        np.testing.assert_allclose(extents, box.extents())

    def test_setters(self) -> None:
        # SETUP
        DIM = 3
        reference_from_box = se3.SE3.exp(np.random.rand(se3.SE3.DOF))
        extents = np.random.rand(DIM)
        box = oribox.OrientedBox(reference_from_box=reference_from_box, extents=extents)

        # ACTION
        new_reference_from_box = se3.SE3.exp(np.random.rand(se3.SE3.DOF))
        new_extents = np.random.rand(DIM)
        box.set_reference_from_box(new_reference_from_box)
        box.set_extents(new_extents)

        # VERIFICATION
        self.assertTrue(new_reference_from_box.is_approx(box.reference_from_box()))
        np.testing.assert_allclose(new_extents, box.extents())


if __name__ == "__main__":
    unittest.main()
