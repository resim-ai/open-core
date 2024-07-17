# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Unit tests for SO3 pybinding
"""

import unittest

import numpy as np

import resim.transforms.python.quaternion as quat
import resim.transforms.python.so3_python as so3


class SO3PythonTest(unittest.TestCase):
    """Unit tests for SO3 pybinding"""

    def setUp(self) -> None:
        """Seed the random number generator"""
        np.random.seed(29)

    def test_dims_dof(self) -> None:
        """Test that DIMS and DOF are correct"""
        self.assertEqual(so3.SO3.DIMS, 3)
        self.assertEqual(so3.SO3.DOF, 3)

    def test_default_constructor(self) -> None:
        """Check that the default constructor, is_approx, and identity work"""
        self.assertTrue(so3.SO3().is_approx(so3.SO3.identity()))

    def test_quaternion(self) -> None:
        """Check that the quaternion constructor works."""
        # SETUP
        vec = np.random.rand(4)
        vec = vec / np.linalg.norm(vec)
        quaternion = quat.Quaternion(vec)

        # ACTION
        rotation = so3.SO3(quaternion)

        # VERIFICATION
        quaternion_out = rotation.quaternion()
        self.assertAlmostEqual(quaternion.w(), quaternion_out.w())
        self.assertAlmostEqual(quaternion.x(), quaternion_out.x())
        self.assertAlmostEqual(quaternion.y(), quaternion_out.y())
        self.assertAlmostEqual(quaternion.z(), quaternion_out.z())

        # Confirm that the quaternion matches up with the SO3
        angle_rad = 2.0 * np.arccos(quaternion.w())
        axis = np.array([quaternion.x(), quaternion.y(), quaternion.z()]) / np.sin(
            0.5 * angle_rad
        )
        self.assertTrue(np.allclose(axis * angle_rad, rotation.log()))

    def test_inverse(self) -> None:
        """Check that inversion works"""
        num_tests = 10
        for _ in range(num_tests):
            # SETUP
            rotation = so3.SO3.exp(np.random.rand(so3.SO3.DOF))

            # ACTION / VERIFICATION
            self.assertTrue(
                (rotation * rotation.inverse()).is_approx(so3.SO3.identity())
            )

    def test_interp(self) -> None:
        """Check that interpolation works"""
        num_tests = 10
        for _ in range(num_tests):
            # SETUP
            rotation = so3.SO3.exp(np.random.rand(so3.SO3.DOF))

            # ACTION / VERIFICATION
            scale = np.random.random()
            interped = rotation.interp(scale)
            self.assertTrue(interped.is_approx(so3.SO3.exp(scale * rotation.log())))

    def test_exp_log(self) -> None:
        """Test that exponential ang log are correct."""
        num_tests = 10
        for _ in range(num_tests):
            # SETUP
            arg = np.random.rand(so3.SO3.DOF)
            # Clamp arg to have ||arg|| < PI
            # This is needed because we might otherwise exit the subset of the
            # domain where exp is invertible.
            epsilon = 1e-3
            arg = (1.0 - epsilon) * arg * (np.pi / max(1.0, np.linalg.norm(arg)))

            # ACTION / VERIFICATION
            test_so3 = so3.SO3.exp(arg)
            self.assertTrue(np.allclose(test_so3.log(), arg))


if __name__ == "__main__":
    unittest.main()
