// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// This file is a companion for the Curves documentation at
// https://docs.resim.ai/curves/

#include <cstdlib>

#include "resim/actor/state/rigid_body_state.hh"
#include "resim/assert/assert.hh"
#include "resim/curves/d_curve.hh"
#include "resim/curves/t_curve.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/visualization/view.hh"

int main(int argc, char **argv) {
  using resim::transforms::SE3;
  using resim::transforms::SO3;
  using RigidBodyState = resim::actor::state::RigidBodyState<SE3>;
  using TCurve = resim::curves::TCurve<SE3>;
  using Frame = resim::transforms::Frame<3>;

  const Frame world_frame{Frame::new_frame()};
  // VIEW(world_frame) << "World frame";

  //////////////////////////////////////////////////////////////////////////////
  // DCurve
  //////////////////////////////////////////////////////////////////////////////

  const Frame d_curve_frame{Frame::new_frame()};
  // VIEW(d_curve_frame) << "DCurve";

  // Make a DCurve that goes straight for one unit, turns left along a unit
  // circle for pi/2 radians, continues straight in the y direction for one
  // unit, and then turns to the right back to its original heading along a unit
  // circle.
  const resim::curves::DCurve<SE3> d_curve{
      SE3::identity(world_frame, d_curve_frame),
      SE3{{1., 0., 0.}, world_frame, d_curve_frame},
      SE3{SO3{M_PI_2, {0., 0., 1.}}, {2., 1., 0.}, world_frame, d_curve_frame},
      SE3{SO3{M_PI_2, {0., 0., 1.}}, {2., 2., 0.}, world_frame, d_curve_frame},
      SE3{{3., 3., 0.}, world_frame, d_curve_frame},
  };

  // Query a point halfway along the third edge:
  const double QUERY_ARC_LENTH = 1.0 + M_PI_2 + 0.5;
  const SE3 reference_from_queried{d_curve.point_at(QUERY_ARC_LENTH)};

  REASSERT(
      reference_from_queried.rotation().is_approx(SO3{M_PI_2, {0., 0., 1.}}));
  REASSERT(reference_from_queried.translation().isApprox(
      Eigen::Vector3d{2., 1.5, 0.}));

  // VIEW(d_curve) << "My DCurve";

  //////////////////////////////////////////////////////////////////////////////
  // TCurve
  //////////////////////////////////////////////////////////////////////////////

  const Frame t_curve_frame{Frame::new_frame()};
  // VIEW(t_curve_frame) << "TCurve";

  // Define some states that we want to pass through
  // A state at the origin with a small forward velocity
  RigidBodyState state_a{SE3::identity(world_frame, t_curve_frame)};
  state_a.set_body_linear_velocity_mps({0.1, 0., 0.});

  // A state at (1, 1, 0) oriented along the x axis with a small forward
  // velocity and a small angular velocity about the x axis.
  RigidBodyState state_b{SE3{{1., 1., 0.}, world_frame, t_curve_frame}};
  state_b.set_body_linear_velocity_mps({0.1, 0., 0.});
  state_b.set_body_angular_velocity_radps({0.1, 0., 0.});

  // A state at (3, 0, 0.5) with a rotation of pi/4 about the z axis
  RigidBodyState state_c{
      SE3{SO3{M_PI_4, {0., 0., 1.}},
          {3., 0., 0.5},
          world_frame,
          t_curve_frame}};
  state_c.set_body_linear_velocity_mps({0.1, 0., 0.});

  // Create a t_curve by getting left two jets from the states:
  const TCurve t_curve{{
      TCurve::Control{
          .time = 0.,
          .point = state_a.ref_from_body_two_jet().left_two_jet(),
      },
      TCurve::Control{
          .time = 10.,
          .point = state_b.ref_from_body_two_jet().left_two_jet(),
      },
      TCurve::Control{
          .time = 20.,
          .point = state_c.ref_from_body_two_jet().left_two_jet(),
      },
  }};

  // Visualize
  // VIEW(t_curve) << "My TCurve";

  return EXIT_SUCCESS;
}
