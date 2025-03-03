# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Unit tests for dice_coefficient.py
"""

import copy
import unittest

import resim.msg.detection_pb2 as Detection

import resim.metrics.dice_coefficient as dc


class DiceCoefficientTest(unittest.TestCase):
    """
    Unit tests for dice_coefficient.py
    """

    def test_compute_dice_coefficient(self) -> None:
        """
        Test that we get the right dice coefficient for a variety of setups.
        """
        box_a = Detection.BoundingBox2D()
        box_a.center_x = 0.0
        box_a.center_y = 0.0
        box_a.size_x = 1.0
        box_a.size_y = 1.0

        num_sliding_samples = 11
        box_b_samples = (
            [
                Detection.BoundingBox2D(
                    center_x=1.0, center_y=1.0, size_x=1.0, size_y=1.0
                ),
                Detection.BoundingBox2D(
                    center_x=0.5, center_y=0.5, size_x=1.0, size_y=1.0
                ),
            ]
            + [
                Detection.BoundingBox2D(
                    center_x=i / (num_sliding_samples - 1),
                    center_y=0.0,
                    size_x=1.0,
                    size_y=1.0,
                )
                for i in range(num_sliding_samples)
            ]
            + [
                Detection.BoundingBox2D(
                    center_x=0.0,
                    center_y=i / (num_sliding_samples - 1),
                    size_x=1.0,
                    size_y=1.0,
                )
                for i in range(num_sliding_samples)
            ]
        )

        expected_sliding_sample_dice = [
            1.0 - i / (num_sliding_samples - 1) for i in range(num_sliding_samples)
        ]
        expected_dice = [0.0, 0.25] + 2 * expected_sliding_sample_dice

        for i, box_b in enumerate(box_b_samples):
            dice = dc.compute_dice_coefficient(box_a, box_b)
            self.assertAlmostEqual(dice, expected_dice[i])

    def test_fails_on_spun_box(self) -> None:
        """
        Test that we currently fail on boxes with non-zero spin.

        We plan to support this functionality in the future at which point this
        test should be removed.
        """
        box = Detection.BoundingBox2D()
        box.center_x = 0.0
        box.center_y = 0.0
        box.size_x = 1.0
        box.size_y = 1.0

        spun_box = copy.copy(box)
        spun_box.theta_rad = 1e-5

        with self.assertRaises(ValueError):
            dc.compute_dice_coefficient(box, spun_box)
        with self.assertRaises(ValueError):
            dc.compute_dice_coefficient(spun_box, box)


if __name__ == "__main__":
    unittest.main()
