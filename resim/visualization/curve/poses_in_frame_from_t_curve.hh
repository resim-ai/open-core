#pragma once

#include <foxglove/PosesInFrame.pb.h>

#include <string>

#include "resim/curves/t_curve.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"

namespace resim::visualization::curve {

// This function samples the given curve producing poses at all times t along
// the curve such that (t - time) is a multiple of pose_period. When called with
// steadily increasing times, this function therefore generates a sequence of
// PosesInFrame messages which produce an "escalator" effect when viewed
// sequentially.
// @param[in] curve - The curve to sample.
// @param[in] frame_id - The frame_id associated with the curve.
// @param[in] time - The time to produce frames at.
// @param[in] pose_period - The distance between adjacent frames.
// @param[in] poses - The message to pack our poses into.
void poses_in_frame_from_t_curve(
    const curves::TCurve<transforms::SE3> &curve,
    const std::string &frame_id,
    time::Timestamp time,
    time::Duration pose_period,
    ::foxglove::PosesInFrame *poses);

}  // namespace resim::visualization::curve
