#pragma once

#include "resim/geometry/oriented_box.hh"
#include "resim/geometry/wireframe.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"

namespace resim::geometry {

// This function computes a bounding box for a given wireframe, without frames.
// The orientation of this bounding box is defined to be the identity and the
// translation is adjusted to be at the geometric center of the box.
// @param[in] wireframe - The wireframe to compute a bounding box for.
// @returns An oriented bounding box for this wireframe.
OrientedBox<transforms::SE3> bounding_box_from_wireframe(
    const Wireframe &wireframe);

// This function is an overload of the above which adds named coordinate frames
// to the resulting bounding box. These are passed in by the user as a given
// reference frame and box frame.
// @param[in] wireframe - The wireframe to compute a bounding box for.
// @param[in] reference_frame - The into frame for the resulting bounding box
//                              transformation.
// @param[in] box_frame - The coordinate frame to use for the new box. Defaults
//                        to a new frame.
// @returns An oriented bounding box for this wireframe.
OrientedBox<transforms::SE3> bounding_box_from_wireframe(
    const Wireframe &wireframe,
    const transforms::Frame<transforms::SE3::DIMS> &reference_frame,
    const transforms::Frame<transforms::SE3::DIMS> &box_frame =
        transforms::Frame<transforms::SE3::DIMS>::new_frame());

}  // namespace resim::geometry
