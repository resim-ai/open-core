#pragma once

#include <initializer_list>

#include "resim/actor/state/rigid_body_state.hh"
#include "resim/curves/t_curve.hh"
#include "resim/curves/two_jet.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"

namespace resim::actor::state {

// Trajectory.
// Describes the state of a six-degrees-of-freedom rigid-body through over time,
// including the pose and the first and second time derivatives.
//
// The Trajectory may be sampled at timestamps or durations, returning
// RigidBodyState<SE3> objects describing the state of the body at that time.
// Trajectories can be constructed from control points consisting of timestamps
// and RigidBodyStates. All points in the trajectory must have the same
// reference frame *and* the same body frame as the trajectory represents the
// path of a persistent body. Thus the Trajectory itself may be thought of as
// having a reference and body frame that can be queried.
//
// Under the hood the trajectory is represented by a TCurve<SE3>. The TCurve
// represents time as a double type. We always start the underlying TCurve at
// a time of 0s to maintain maximum accuracy in the outbound conversion from
// double to time::Timestamp when we sample points.
class Trajectory {
 public:
  using SE3 = transforms::SE3;
  using Frame = transforms::Frame<SE3::DIMS>;

  // Data for a trajectory control point.
  struct Control {
    // Timestamp of the control point.
    time::Timestamp at_time;
    // RigidBodyState specifying he control point.
    RigidBodyState<SE3> state;
  };

  // Create and empty trajectory to be populated.
  Trajectory() = default;

  // Create a trajectory directly from a TCurve<SE3>
  Trajectory(curves::TCurve<SE3> curve, time::Timestamp start_time);

  // Create a trajectory from a list of control points
  Trajectory(std::initializer_list<Control> points);

  // Append a single control point to the back of the trajectory.
  // Note the timestamps of control points must be strictly increasing. Errors
  // will be thrown on invalid timestamps.
  void append(const Control &point);

  // Append a list of control points to the back of the trajectory.
  // Note the timestamps of control points must be strictly increasing. Errors
  // will be thrown on invalid timestamps.
  void append(std::initializer_list<Control> points);

  // Query the state of the rigid body at a given timestamp.
  // Note the timestamp must be within the valid range of the trajectory. Errors
  // will be thrown on invalid timestamps.
  RigidBodyState<SE3> point_at(const time::Timestamp &at_time) const;

  // Query the state of the rigid body at a given time duration from the start.
  // Note the timestamp must be within the valid range of the trajectory. Errors
  // will be thrown on invalid timestamps.
  RigidBodyState<SE3> point_at(const time::Duration &from_start) const;

  // Getter for the underlying TCurve.
  const curves::TCurve<SE3> &curve() const { return curve_; }

  // Get the timestamp at which the trajectory begins.
  const time::Timestamp &start_time() const;

  // Get the timestamp at which the trajectory ends.
  time::Timestamp end_time() const;

  // Get the time duration of the trajectory.
  time::Duration time_duration() const;

  // Query the reference frame of the trajectory's points.
  const Frame &reference_frame() const;

  // Query the body frame of the trajectory's points.
  const Frame &body_frame() const;

 private:
  curves::TCurve<SE3> curve_;
  time::Timestamp start_time_;
};

}  // namespace resim::actor::state
