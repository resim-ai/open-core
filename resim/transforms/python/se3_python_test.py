# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Unit tests for SE3 pybinding
"""

import unittest

import numpy as np
import resim.transforms.python.se3_python as se3
import resim.transforms.python.so3_python as so3


class SE3PythonTest(unittest.TestCase):
    """Unit tests for SE3 pybinding"""

    def setUp(self):
        """Seed the random number generator"""
        np.random.seed(29)

    def test_dims_dof(self):
        """Test that DIMS and DOF are correct"""
        self.assertEqual(se3.SE3.DIMS, 3)
        self.assertEqual(se3.SE3.DOF, 6)

    def test_default_constructor(self):
        """Check that the default constructor, is_approx, and identity work"""
        self.assertTrue(se3.SE3().is_approx(se3.SE3.identity()))

    def test_inverse(self):
        """Check that inversion works"""
        num_tests = 10
        for _ in range(num_tests):
            # SETUP
            rotation = se3.SE3.exp(np.random.rand(se3.SE3.DOF))

            # ACTION / VERIFICATION
            self.assertTrue(
                (rotation *
                 rotation.inverse()).is_approx(
                    se3.SE3.identity()))

    def test_interp(self):
        """Check that interpolation works"""
        num_tests = 10
        for _ in range(num_tests):
            # SETUP
            rotation = se3.SE3.exp(np.random.rand(se3.SE3.DOF))

            # ACTION / VERIFICATION
            scale = np.random.random()
            interped = rotation.interp(scale)
            self.assertTrue(
                interped.is_approx(
                    se3.SE3.exp(
                        scale *
                        rotation.log())))

    def test_exp_log(self):
        """Test that exponential ang log are correct."""
        num_tests = 10
        for _ in range(num_tests):
            # SETUP
            arg = np.random.rand(se3.SE3.DOF)
            # Clamp arg to have ||arg|| < PI
            # This is needed because we might otherwise exit the subset of the
            # domain where exp is invertible.
            epsilon = 1e-3
            arg = (1. - epsilon) * arg * (np.pi / max(1., np.linalg.norm(arg)))

            # ACTION / VERIFICATION
            test_se3 = se3.SE3.exp(arg)
            self.assertTrue(np.allclose(test_se3.log(), arg))

    def test_rotation_and_translation(self):
        """Test that rotation and translation getters work."""
        num_tests = 10
        for _ in range(num_tests):
            # SETUP
            rotation = so3.SO3.exp(np.random.rand(so3.SO3.DOF))
            translation = np.random.rand(se3.SE3.DIMS)

            pose = se3.SE3(rotation, translation)

            # ACTION / VERIFICATION
            self.assertTrue(rotation.is_approx(pose.rotation()))
            self.assertTrue(np.allclose(translation, pose.translation()))


if __name__ == '__main__':
    unittest.main()
