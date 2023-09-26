

import resim.msg.detection_pb2 as Detection


def _intersection(
        box_a: Detection.BoundingBox2D,
        box_b: Detection.BoundingBox2D):
    if box_a.theta_rad != 0. != box_b.theta_rad:
        raise ValueError("Rotated boxes not supported!")

    # Since we're not allowing rotations, the intersection area is simple:

    intersection_max_x = min(box_a.center_x + 0.5 * box_a.size_x,
                             box_b.center_x + 0.5 * box_b.size_x)
    intersection_min_x = max(box_a.center_x - 0.5 * box_a.size_x,
                             box_b.center_x - 0.5 * box_b.size_x)
    intersection_max_y = min(box_a.center_y + 0.5 * box_a.size_y,
                             box_b.center_y + 0.5 * box_b.size_y)
    intersection_min_y = max(box_a.center_y - 0.5 * box_a.size_y,
                             box_b.center_y - 0.5 * box_b.size_y)
    return max((intersection_max_x - intersection_min_x), 0.) * \
        max((intersection_max_y - intersection_min_y), 0.)


def _area(box: Detection.BoundingBox2D):
    return box.size_x * box.size_y


def compute_dice_coefficient(
        box_a: Detection.BoundingBox2D,
        box_b: Detection.BoundingBox2D):
    return 2.0 * _intersection(box_a, box_b) / (_area(box_a) + _area(box_b))
