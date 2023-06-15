
#pragma once

#include "resim/curves/t_curve.hh"
#include "resim/transforms/frame.hh"

namespace resim::curves::testing {

// This helper creates a circular TCurve that follows the unit circle with
// control points at (1, 0), (0, 1), (-1, 0), (0, -1), and back at (1.0). The
// curve has a constant linear velocity of 1 in the x direction and a constant
// angular velocity of 1 in the z direction and consequently completes its orbit
// over a time of 2 * M_PI.
// @param[in] into - The into frame for the control points.
// @param[in] from - The from frame for the control points (i.e. the reference
//                   frame for this curve).
curves::TCurve<transforms::SE3> make_circle_curve(
    const transforms::Frame<transforms::SE3::DIMS> &into =
        transforms::Frame<transforms::SE3::DIMS>::new_frame(),
    const transforms::Frame<transforms::SE3::DIMS> &from =
        transforms::Frame<transforms::SE3::DIMS>::new_frame());

}  // namespace resim::curves::testing
