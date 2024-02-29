# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Unit tests for Polygon Distance pybinding
"""

import unittest

import numpy as np
import resim.geometry.python.polygon_distance_python as polydist


class PolygonDistanceTest(unittest.TestCase):
    """Unit tests for Polygon Distance pybinding"""


    def test_polygon_distance(self) -> None:
        poly_a = [np.array([-1., -0.5]),
                  np.array([1., -0.5]),
                  np.array([0., 1.0])]
        poly_b = [np.array([-1., 0.5]),
                  np.array([1., 0.5]),
                  np.array([0., -1.0])]
        poly_c = [np.array([-1., 1.5]),
                  np.array([1., 1.5]),
                  np.array([1., 2.5]),
                  np.array([-1., 2.5])]

        self.assertEqual(polydist.polygon_distance(poly_a, poly_b), 0.)
        self.assertEqual(polydist.polygon_distance(poly_a, poly_c), 0.5)
        self.assertEqual(polydist.polygon_distance(poly_b, poly_c), 1.0)



if __name__ == '__main__':
    unittest.main()
