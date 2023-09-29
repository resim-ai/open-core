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


class SE3PythonTest(unittest.TestCase):
    """Unit tests for SE3 pybinding"""

    def setUp(self):
        """Seed the random number generator"""
        np.random.seed(29)

    def test_exp_log(self):
        """Test that exponential ang log are correct."""
        num_tests = 100
        for _ in range(num_tests):
            arg = np.random.rand(se3.SE3.DOF)
            # Clamp arg to have ||arg|| <= PI
            # This is needed because we might otherwise exit the subset of the
            # domain where exp is invertible.
            arg = arg * (np.pi / max(1., np.linalg.norm(arg)))

            test_se3 = se3.SE3.exp(arg)
            self.assertAlmostEqual(np.linalg.norm(test_se3.log() - arg), 0.0)


if __name__ == '__main__':
    unittest.main()
