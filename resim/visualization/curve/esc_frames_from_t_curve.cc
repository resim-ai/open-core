// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/curve/esc_frames_from_t_curve.hh"

#include "resim/time/sample_interval.hh"
#include "resim/visualization/curve/poses_in_frame_from_t_curve.hh"
#include "resim/visualization/foxglove/pose_to_foxglove.hh"

namespace resim::visualization::curve {

EscalatorFrames esc_frames_from_t_curve(
    const curves::TCurve<transforms::SE3> &curve,
    const std::string &frame_id,
    const time::Duration pose_period,
    const time::Duration publish_rate) {
  EscalatorFrames poses_in_frame;
  esc_frames_from_t_curve(
      curve,
      frame_id,
      pose_period,
      publish_rate,
      InOut(poses_in_frame));
  return poses_in_frame;
}

void esc_frames_from_t_curve(
    const curves::TCurve<transforms::SE3> &curve,
    const std::string &frame_id,
    const time::Duration pose_period,
    const time::Duration publish_rate,
    InOut<EscalatorFrames> poses_in_frame) {
  // Create PosesInFrame to visualize control points.
  const time::Timestamp start_time{time::as_duration(curve.start_time())};
  const time::Timestamp end_time{start_time + pose_period};

  time::sample_interval(
      start_time,
      end_time,
      publish_rate,
      [&](const time::Timestamp time) {
        curve::poses_in_frame_from_t_curve(
            curve,
            frame_id,
            time,
            pose_period,
            &(*poses_in_frame)[time]);
      });
}

}  // namespace resim::visualization::curve
