#pragma once

#include <foxglove/PosesInFrame.pb.h>

#include <map>
#include <utility>

#include "resim/curves/t_curve.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"

namespace resim::visualization::curve {

using EscalatorFrames = std::map<time::Timestamp, ::foxglove::PosesInFrame>;

// Samples the TCurve over time and displays a sequence of poses over time,
// which when viewed in sequence, produces an escalator-like effect.
// @param[in] curve - The curve to sample.
// @param[in] frame_id - The frame_id associated with the curve reference frame.
// @param[in] pose_period - The time between adjacent frames.
// @param[in] publish_rate - The duration between published foxglove messages.
// @returns - A map of timestamps and the poses to view at
//            each time stamp.
EscalatorFrames esc_frames_from_t_curve(
    const curves::TCurve<transforms::SE3> &curve,
    const std::string &frame_id,
    time::Duration pose_period,
    time::Duration publish_rate);

// If the user would like to load multiple curves into an EscalatorFrames map
// in order to view frames on each curve at the same time, then they may use
// this overload which does not create the map but takes an InOut reference to
// a pre-existing map.
// @param[in] curve - The curve to sample.
// @param[in] frame_id - The frame_id associated with the curve reference frame.
// @param[in] pose_period - The time between adjacent frames.
// @param[in] publish_rate - The duration between published foxglove messages.
// @param[in-out] poses_in_frame - A map of timestamps and the poses to view at
//                                 each time stamp.
void esc_frames_from_t_curve(
    const curves::TCurve<transforms::SE3> &curve,
    const std::string &frame_id,
    time::Duration pose_period,
    time::Duration publish_rate,
    InOut<EscalatorFrames> poses_in_frame);

}  // namespace resim::visualization::curve
