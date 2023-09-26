# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
Unit tests for dice_coefficient.py
"""

import unittest

import resim.msg.detection_pb2 as Detection
import resim.metrics.dice_coefficient as dc


class DiceCoefficientTest(unittest.TestCase):
    """
    Unit tests for dice_coefficient.py
    """

    def test_compute_dice_coefficient(self):
        box_a = Detection.BoundingBox2D()
        box_a.center_x = 0.
        box_a.center_y = 0.
        box_a.size_x = 1.
        box_a.size_y = 1.

        num_sliding_samples = 11
        box_b_samples = [
            Detection.BoundingBox2D(
                center_x=1.,
                center_y=1.,
                size_x=1.,
                size_y=1.),
            Detection.BoundingBox2D(
                center_x=0.5,
                center_y=0.5,
                size_x=1.,
                size_y=1.),
        ] + [
            Detection.BoundingBox2D(
                center_x=i / (num_sliding_samples - 1),
                center_y=0.,
                size_x=1.,
                size_y=1.) for i in range(num_sliding_samples)
        ] + [
            Detection.BoundingBox2D(
                center_x=0.,
                center_y=i / (num_sliding_samples - 1),
                size_x=1.,
                size_y=1.) for i in range(num_sliding_samples)]

        expected_dice = [0., 0.25] + 2 * [1. - i / \
            (num_sliding_samples - 1) for i in range(num_sliding_samples)]

        for i in range(len(box_b_samples)):
            dice = dc.compute_dice_coefficient(box_a, box_b_samples[i])
            self.assertAlmostEqual(dice, expected_dice[i])


if __name__ == '__main__':
    unittest.main()
