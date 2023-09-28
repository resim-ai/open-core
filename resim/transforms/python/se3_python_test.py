# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import unittest

import numpy as np
import resim.transforms.python.se3_python as se3


class SE3PythonTest(unittest.TestCase):
    """Unit tests for SE3 pybinding"""

    def setUp(self):
        np.random.seed(29)

    def test_exp_log(self):
        NUM_TESTS = 100
        for _ in range(NUM_TESTS):
            arg = np.random.rand(se3.SE3.DOF)
            # Clamp arg to have ||arg|| <= PI
            # This is needed because we might otherwise exit the subset of the
            # domain where exp is inverible.
            arg = arg * (np.pi / max(1., np.linalg.norm(arg)))

            test_se3 = se3.SE3.exp(arg)
            TOLERANCE = 1e-10
            self.assertAlmostEqual(np.linalg.norm(test_se3.log() - arg), 0.0)


if __name__ == '__main__':
    unittest.main()
