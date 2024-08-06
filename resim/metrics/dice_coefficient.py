# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.


"""
A module for computing Sorensen-Dice coefficient.
"""

import resim.msg.detection_pb2 as Detection


def _intersection(
    box_a: Detection.BoundingBox2D, box_b: Detection.BoundingBox2D
) -> float:
    """Helper to compute the intersection area between 2D bounding boxes."""
    if box_a.theta_rad != 0.0 or box_b.theta_rad != 0.0:
        raise ValueError("Rotated boxes not supported!")

    # Since we're not allowing rotations, the intersection area is simple:

    intersection_max_x: float = min(
        box_a.center_x + 0.5 * box_a.size_x, box_b.center_x + 0.5 * box_b.size_x
    )
    intersection_min_x: float = max(
        box_a.center_x - 0.5 * box_a.size_x, box_b.center_x - 0.5 * box_b.size_x
    )
    intersection_max_y: float = min(
        box_a.center_y + 0.5 * box_a.size_y, box_b.center_y + 0.5 * box_b.size_y
    )
    intersection_min_y: float = max(
        box_a.center_y - 0.5 * box_a.size_y, box_b.center_y - 0.5 * box_b.size_y
    )
    return max((intersection_max_x - intersection_min_x), 0.0) * max(
        (intersection_max_y - intersection_min_y), 0.0
    )


def _area(box: Detection.BoundingBox2D) -> float:
    """Helper to compute the area of a 2D box."""
    size_x: float = box.size_x
    size_y: float = box.size_y
    return size_x * size_y


def compute_dice_coefficient(
    box_a: Detection.BoundingBox2D, box_b: Detection.BoundingBox2D
) -> float:
    """
    Compute the Sorensen-Dice coefficient.

    Args:
        box_a: The first box.
        box_b: The second box.

    Returns:
        The Sorensen-Dice coefficient between box_a and box_b.
    """
    return 2.0 * _intersection(box_a, box_b) / (_area(box_a) + _area(box_b))
