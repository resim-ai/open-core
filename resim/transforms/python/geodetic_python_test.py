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
import resim.transforms.python.so3_python as so3


class GeodeticPythonTest(unittest.TestCase):
    """Unit tests for Geodetic pybinding"""

    def setUp(self) -> None:
        """Seed the random number generator"""
        np.random.seed(29)

    def test_roundtrip(self) -> None:
        num_tests = 100

        for _ in range(num_tests):
            g = geodetic.Geodetic(
                latitude_deg=np.random.uniform(0., 90.),
                longitude_deg=np.random.uniform(-180., 180.),
                altitude_m=np.random.uniform(0., 1e4))

            ecef_position = geodetic.ecef_position_from_geodetic(g)

            round_tripped = geodetic.geodetic_from_ecef_position(ecef_position)
            self.assertAlmostEqual(
                g.latitude_deg(),
                round_tripped.latitude_deg())
            self.assertAlmostEqual(
                g.longitude_deg(),
                round_tripped.longitude_deg())
            self.assertAlmostEqual(g.altitude_m(), round_tripped.altitude_m())
            self.assertAlmostEqual(
                g.altitude_ft(), round_tripped.altitude_ft())

    def test_setters(self) -> None:
        g = geodetic.Geodetic()

        num_tests = 100
        for _ in range(num_tests):
            latitude_deg = np.random.uniform(0., 90.)
            g.set_latitude_deg(latitude_deg)
            self.assertEqual(latitude_deg, g.latitude_deg())

            longitude_deg = np.random.uniform(-180., 180.)
            g.set_longitude_deg(longitude_deg)
            self.assertEqual(longitude_deg, g.longitude_deg())

            altitude_m = np.random.uniform(0., 1e4)
            g.set_altitude_m(altitude_m)
            self.assertEqual(altitude_m, g.altitude_m())

            altitude_ft = np.random.uniform(0., 3e4)
            g.set_altitude_ft(altitude_ft)
            self.assertAlmostEqual(altitude_ft, g.altitude_ft())

    def test_with_rotation_roundtrip(self) -> None:
        num_tests = 100

        for _ in range(num_tests):
            g = geodetic.GeodeticWithRotation(
                geodetic=geodetic.Geodetic(
                    latitude_deg=np.random.uniform(0., 90.),
                    longitude_deg=np.random.uniform(-180., 180.),
                    altitude_m=np.random.uniform(0., 1e4)),
                rotation=so3.SO3.exp(np.random.uniform(-1., 1., 3))
            )

            ecef_pose = geodetic.ecef_from_body_from_geodetic_with_rotation(g)

            round_tripped = geodetic.geodetic_with_rotation_from_ecef_from_body(
                ecef_pose)
            self.assertAlmostEqual(
                g.geodetic.latitude_deg(),
                round_tripped.geodetic.latitude_deg())
            self.assertAlmostEqual(
                g.geodetic.longitude_deg(),
                round_tripped.geodetic.longitude_deg())
            self.assertAlmostEqual(
                g.geodetic.altitude_m(),
                round_tripped.geodetic.altitude_m())
            self.assertAlmostEqual(
                g.geodetic.altitude_ft(),
                round_tripped.geodetic.altitude_ft())
            self.assertTrue(g.rotation.is_approx(round_tripped.rotation))


if __name__ == '__main__':
    unittest.main()
