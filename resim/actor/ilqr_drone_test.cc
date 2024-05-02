// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/actor/ilqr_drone.hh"

#include <gtest/gtest.h>

#include "resim/actor/actor_id.hh"
#include "resim/math/is_approx.hh"
#include "resim/simulator/standard_frames.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"

namespace resim::actor {
namespace {
using Vec3 = Eigen::Vector3d;
}

TEST(ILQRDroneTest, TestInitialization) {
  // SETUP
  constexpr size_t SEED{9U};
  std::mt19937 rng{SEED};
  const ActorId id{ActorId::new_uuid()};
  const Vec3 initial_position{5.0 * testing::random_vector<Vec3>(rng)};
  const Vec3 goal_position{5.0 * testing::random_vector<Vec3>(rng)};
  const double velocity_cost = 3.0;

  constexpr time::Timestamp START_TIME{time::as_duration(1.0)};

  // ACTION
  ILQRDrone drone{id, initial_position, goal_position, velocity_cost};

  EXPECT_FALSE(drone.observable_state().is_spawned);
  drone.simulate_forward(START_TIME);

  auto state = drone.observable_state();
  EXPECT_TRUE(state.is_spawned);
  EXPECT_TRUE(state.state.ref_from_body().is_approx(transforms::SE3(
      initial_position,
      simulator::SCENE_FRAME,
      transforms::Frame<3>{id})));
  EXPECT_TRUE(math::is_approx(
      state.state.ref_from_body_two_jet().d_ref_from_frame(),
      transforms::SE3::TangentVector::Zero()));
  EXPECT_TRUE(math::is_approx(
      state.state.ref_from_body_two_jet().d2_ref_from_frame(),
      transforms::SE3::TangentVector::Zero()));
}

TEST(ILQRDroneTest, TestNavigateToGoal) {
  // SETUP
  constexpr size_t SEED{9U};
  std::mt19937 rng{SEED};
  const ActorId id{ActorId::new_uuid()};
  const Vec3 initial_position{5.0 * testing::random_vector<Vec3>(rng)};
  const Vec3 goal_position{5.0 * testing::random_vector<Vec3>(rng)};
  const double velocity_cost = 3.0;

  constexpr time::Timestamp START_TIME{time::as_duration(1.0)};
  constexpr time::Timestamp END_TIME{time::as_duration(10.0)};

  // ACTION
  ILQRDrone drone{id, initial_position, goal_position, velocity_cost};

  const time::Duration DT{time::as_duration(0.05)};
  for (auto time = START_TIME; time <= END_TIME; time += DT) {
    drone.simulate_forward(time);
  }
  auto state = drone.observable_state();

  // We can't expect error to be precisely zero as the cost is
  // quadratic near the goal.
  constexpr double TOLERANCE = 1e-2;
  EXPECT_TRUE(state.is_spawned);
  EXPECT_TRUE(state.state.ref_from_body().is_approx(
      transforms::SE3(
          goal_position,
          simulator::SCENE_FRAME,
          transforms::Frame<3>{id}),
      TOLERANCE));

  EXPECT_TRUE(math::is_approx(
      state.state.ref_from_body_two_jet().d_ref_from_frame(),
      transforms::SE3::TangentVector::Zero(),
      TOLERANCE));
  EXPECT_TRUE(math::is_approx(
      state.state.ref_from_body_two_jet().d2_ref_from_frame(),
      transforms::SE3::TangentVector::Zero(),
      TOLERANCE));
}

}  // namespace resim::actor
