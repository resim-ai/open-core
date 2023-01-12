#include "resim_core/actor/state/trajectory.hh"

#include <glog/logging.h>

#include "resim_core/actor/state/rigid_body_state.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::actor::state {

namespace {
using FSE3 = transforms::FSE3;
using Frame = transforms::Frame<FSE3::DIMS>;
}  // namespace

Trajectory::Trajectory(std::initializer_list<Control> points) {
  append(points);
}

void Trajectory::append(const Control &point) {
  if (curve_.control_pts().empty()) {
    // The first point defines the trajectory start time.
    start_time_ = point.at_time;
  } else {
    // The first point defines the trajectory reference and body frames.
    // All subsequent points must have matching frames. The TCurve checks for
    // reference frame consistency already so here we add an additional check
    // for body frame consistency.
    constexpr auto FRAME_ERR =
        "All control points must have the same body frame";
    const bool frame_condition =
        point.state.ref_from_body().from() == body_frame();
    CHECK(frame_condition) << FRAME_ERR;
  }
  // Convert the point to a valid format for the TCurve object.
  double time = time::as_seconds(point.at_time - start_time());
  curve_.append({time, point.state.ref_from_body_two_jet().left_two_jet()});
}

void Trajectory::append(std::initializer_list<Control> points) {
  for (const Control &point : points) {
    append(point);
  }
}

RigidBodyState<FSE3> Trajectory::point_at(
    const time::Timestamp &at_time) const {
  const double time = time::as_seconds(at_time - start_time());
  return RigidBodyState<FSE3>(
      curve_.point_at(time, body_frame()).right_two_jet());
}

RigidBodyState<FSE3> Trajectory::point_at(
    const time::Duration &from_start) const {
  const double time = time::as_seconds(from_start);
  return RigidBodyState<FSE3>(curve_.point_at(time).right_two_jet());
}

const time::Timestamp &Trajectory::start_time() const { return start_time_; }

time::Timestamp Trajectory::end_time() const {
  return start_time_ + this->time_duration();
}

time::Duration Trajectory::time_duration() const {
  return time::as_duration(curve_.control_pts().back().time);
}

const Frame &Trajectory::reference_frame() const {
  constexpr auto EMPTY_ERR =
      "Cannot query the frame of a trajectory with no control points";
  CHECK(!curve_.control_pts().empty()) << EMPTY_ERR;
  return curve_.control_pts().back().point.frame_from_ref().from();
}

const Frame &Trajectory::body_frame() const {
  constexpr auto EMPTY_ERR =
      "Cannot query the frame of a trajectory with no control points";
  CHECK(!curve_.control_pts().empty()) << EMPTY_ERR;
  return curve_.control_pts().back().point.frame_from_ref().into();
}

}  // namespace resim::actor::state
