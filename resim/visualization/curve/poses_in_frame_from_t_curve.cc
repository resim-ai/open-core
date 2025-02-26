
#include "resim/assert/assert.hh"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/visualization/curve/poses_in_frame_from_t_curve.hh"
#include "resim/visualization/foxglove/pose_to_foxglove.hh"

namespace resim::visualization::curve {

namespace {
using time::Timestamp;
using transforms::SE3;
using TCurve = curves::TCurve<SE3>;
using visualization::foxglove::pack_into_foxglove;
}  // namespace

void poses_in_frame_from_t_curve(
    const TCurve &curve,
    const std::string &frame_id,
    time::Timestamp time,
    time::Duration pose_period,
    ::foxglove::PosesInFrame *const poses) {
  REASSERT(poses != nullptr, "Can't pack into invalid PosesInFrame!");

  const Timestamp start_time{time::as_duration(curve.start_time())};
  const Timestamp end_time{time::as_duration(curve.end_time())};

  resim::time::proto::pack(time, poses->mutable_timestamp());
  // Set the frame id to the reference frame of the TCurve.
  poses->set_frame_id(frame_id);

  const Timestamp first_frame_time{
      start_time +
      time::Duration{(time - start_time).count() % pose_period.count()}};

  for (Timestamp frame_time = first_frame_time; frame_time < end_time;
       frame_time += pose_period) {
    const auto scene_from_frame{
        curve.point_at(time::as_seconds(frame_time.time_since_epoch()))
            .frame_from_ref()
            .inverse()};
    pack_into_foxglove(scene_from_frame, poses->add_poses());
  }
}

}  // namespace resim::visualization::curve
