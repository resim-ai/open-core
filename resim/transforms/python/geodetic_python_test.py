# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Unit tests for geodetic pybinding
"""

import unittest

import numpy as np
import resim.transforms.python.geodetic_python as geodetic

class GeodeticPythonTest(unittest.TestCase):
    """Unit tests for Geodetic pybinding"""

    def setUp(self) -> None:
        """Seed the random number generator"""
        np.random.seed(29)


    def test_foo(self) -> None:
        g = geodetic.Geodetic()
        g.set_latitude_deg(45.0)
        g.set_longitude_deg(46.0)
        g.set_altitude_ft(67.0)

        ecef_position = geodetic.ecef_position_from_geodetic(g)

        round_tripped = geodetic.geodetic_from_ecef_position(ecef_position)
        print(round_tripped.latitude_deg())
        print(round_tripped.longitude_deg())
        print(round_tripped.altitude_ft())        

        
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
