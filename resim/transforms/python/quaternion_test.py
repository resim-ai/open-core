# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Unit tests for Quaternion pybinding
"""

import unittest

import numpy as np
import resim.transforms.python.quaternion as quat


class QuaternionTest(unittest.TestCase):
    """Unit tests for Quaternion pybinding"""

    def setUp(self):
        """Seed the random number generator"""
        np.random.seed(29)

    def test_quaternion(self):
        """Test that we can construct and access quaternions."""
        # SETUP
        quat_dof = 4
        vec = np.random.rand(quat_dof)
        # Normalize
        vec = vec / np.linalg.norm(vec)

        # ACTION
        quaternion = quat.Quaternion(
            x=vec[0],
            y=vec[1],
            z=vec[2],
            w=vec[3])

        # VERIFICATION
        self.assertEqual(quaternion.x(), vec[0])
        self.assertEqual(quaternion.y(), vec[1])
        self.assertEqual(quaternion.z(), vec[2])
        self.assertEqual(quaternion.w(), vec[3])




if __name__ == '__main__':
    unittest.main()
