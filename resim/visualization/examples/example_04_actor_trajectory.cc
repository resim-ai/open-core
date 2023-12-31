// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <vector>

#include "resim/actor/state/rigid_body_state.hh"
#include "resim/actor/state/trajectory.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/uuid.hh"
#include "resim/visualization/view.hh"

namespace {
using resim::transforms::SE3;
using resim::transforms::SO3;
using Frame = resim::transforms::Frame<SE3::DIMS>;
using resim::actor::state::Trajectory;
using RigidBodyState = resim::actor::state::RigidBodyState<SE3>;
using Eigen::Vector3d;

// Define some useful constants for use in example
const Frame REF_FRAME = Frame::new_frame();
const Frame BOD_FRAME = Frame::new_frame();
constexpr unsigned int NUM_CTRL = 3;
const resim::time::Timestamp ZERO_TIME;
// clang-format off
constexpr std::array<double, NUM_CTRL> DEFAULT_TIMES{
     0.0,
    16.5,
    33.0};
const std::array<Vector3d, NUM_CTRL> WAYPOINTS{{
    {  0., -3., 0.},
    {8.,   0., 0.},
    {  0.,  3., 0.}}};
// clang-format on

constexpr std::array<resim::time::Timestamp, NUM_CTRL> timestamps() {
  std::array<resim::time::Timestamp, NUM_CTRL> timestamps;
  std::transform(
      DEFAULT_TIMES.cbegin(),
      DEFAULT_TIMES.cend(),
      timestamps.begin(),
      [](const double t) { return ZERO_TIME + resim::time::as_duration(t); });
  return timestamps;
}

}  // namespace

Trajectory out_and_back_banked() {
  // Point 0.
  SE3 ref_from_0(SO3::identity(), WAYPOINTS.at(0), REF_FRAME, BOD_FRAME);
  RigidBodyState state_0(ref_from_0);
  constexpr double FWD_V_MPS = 5.;
  state_0.set_body_linear_velocity_mps({FWD_V_MPS, 0., 0.});
  // Point 1.
  SE3 ref_from_1(
      (SO3::exp(M_PI_2 * Vector3d::UnitZ()) *
       SO3::exp(-M_PI_2 * Vector3d::UnitX())),
      WAYPOINTS.at(1),
      REF_FRAME,
      BOD_FRAME);
  RigidBodyState state_1(ref_from_1);
  constexpr double TURN_FWD_V_MPS = 7.;
  constexpr double TURN_R_M = 150.;
  const double turn_ang_v_radps = sqrt(TURN_FWD_V_MPS / TURN_R_M);
  state_1.set_body_linear_velocity_mps({TURN_FWD_V_MPS, 0., 0.});
  state_1.set_body_angular_velocity_radps({0., -turn_ang_v_radps, 0.});
  // Point 2.
  SE3 ref_from_2(
      SO3::exp(M_PI * Vector3d::UnitZ()),
      WAYPOINTS.at(2),
      REF_FRAME,
      BOD_FRAME);
  RigidBodyState state_2(ref_from_2);
  state_2.set_body_linear_velocity_mps({FWD_V_MPS, 0., 0.});

  const auto times = timestamps();
  return Trajectory{
      {times.at(0), state_0},
      {times.at(1), state_1},
      {times.at(2), state_2}};
}

int main(int argc, char* argv[]) {
  // In this example, we will explore creating a sample drone
  // trajectory that makes a banked curve and returns to its start location
  Trajectory banked_curve = out_and_back_banked();
  // Visualize the trajectory: note the alternative syntax for VIEW
  VIEW(REF_FRAME, "world");
  VIEW(BOD_FRAME, "body");
  VIEW(banked_curve, "banked_curve");

  // Now, digging into the code that generates this trajectory,
  // we can also view some of the underlying components: the first control
  // point:
  SE3 ref_from_0(SO3::identity(), WAYPOINTS.at(0), REF_FRAME, BOD_FRAME);
  VIEW(ref_from_0) << "ref_from_0_control";

  return EXIT_SUCCESS;
}
