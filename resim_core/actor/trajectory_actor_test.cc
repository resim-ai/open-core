#include "resim_core/actor/trajectory_actor.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <variant>

#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/geometry.hh"
#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/curves/test_helpers.hh"
#include "resim_core/simulator/standard_frames.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"

namespace resim::actor {

TEST(TrajectoryActorTest, TestConstruction) {
  // SETUP
  const transforms::Frame<3> actor_frame{transforms::Frame<3>::new_frame()};
  const curves::TCurve<transforms::FSE3> curve{
      curves::testing::make_circle_curve(simulator::SCENE_FRAME, actor_frame)};

  constexpr time::Timestamp START_TIME{std::chrono::seconds(3)};

  const state::Trajectory trajectory{curve, START_TIME};
  const ActorId id{ActorId::new_uuid()};

  // ACTION
  TrajectoryActor actor{id, trajectory};

  // VERIFICATION
  EXPECT_EQ(actor.id(), id);
}

TEST(TrajectoryActorTest, TestSimulateForwardBeforeSpawn) {
  // SETUP
  const transforms::Frame<3> actor_frame{transforms::Frame<3>::new_frame()};
  const curves::TCurve<transforms::FSE3> curve{
      curves::testing::make_circle_curve(simulator::SCENE_FRAME, actor_frame)};

  constexpr time::Timestamp START_TIME{std::chrono::seconds(3)};

  const state::Trajectory trajectory{curve, START_TIME};
  const ActorId id{ActorId::new_uuid()};
  TrajectoryActor actor{id, trajectory};

  // ACTION
  constexpr time::Timestamp BEFORE_SPAWN{
      START_TIME - std::chrono::nanoseconds(1)};
  actor.simulate_forward(BEFORE_SPAWN);

  // VERIFICATION
  const state::ObservableState state{actor.observable_state()};
  EXPECT_FALSE(state.is_spawned);
  EXPECT_EQ(state.time_of_validity, BEFORE_SPAWN);
  EXPECT_EQ(state.id, actor.id());
  EXPECT_EQ(actor.current_time(), BEFORE_SPAWN);
}

TEST(TrajectoryActorTest, TestSimulateForwardAfterSpawn) {
  // SETUP
  const transforms::Frame<3> actor_frame{transforms::Frame<3>::new_frame()};
  const curves::TCurve<transforms::FSE3> curve{
      curves::testing::make_circle_curve(simulator::SCENE_FRAME, actor_frame)};

  constexpr time::Timestamp START_TIME{std::chrono::seconds(3)};

  const state::Trajectory trajectory{curve, START_TIME};
  const ActorId id{ActorId::new_uuid()};
  TrajectoryActor actor{id, trajectory};

  // ACTION
  const time::Timestamp in_trajectory{
      trajectory.start_time() + trajectory.time_duration() / 2};
  actor.simulate_forward(in_trajectory);

  // VERIFICATION
  const state::ObservableState state{actor.observable_state()};
  EXPECT_TRUE(state.is_spawned);
  EXPECT_EQ(state.time_of_validity, in_trajectory);
  EXPECT_EQ(state.id, actor.id());
  EXPECT_EQ(actor.current_time(), in_trajectory);
  EXPECT_TRUE(state.state.ref_from_body_two_jet().is_approx(
      trajectory.point_at(in_trajectory).ref_from_body_two_jet()));
}

TEST(TrajectoryActorTest, TestSimulateForwardAfterDespawn) {
  // SETUP
  const transforms::Frame<3> actor_frame{transforms::Frame<3>::new_frame()};
  const curves::TCurve<transforms::FSE3> curve{
      curves::testing::make_circle_curve(simulator::SCENE_FRAME, actor_frame)};

  constexpr time::Timestamp START_TIME{std::chrono::seconds(3)};

  const state::Trajectory trajectory{curve, START_TIME};
  const ActorId id{ActorId::new_uuid()};
  TrajectoryActor actor{id, trajectory};

  // ACTION
  const time::Timestamp after_despawn{
      trajectory.end_time() + std::chrono::nanoseconds(1)};
  actor.simulate_forward(after_despawn);

  // VERIFICATION
  const state::ObservableState state{actor.observable_state()};
  EXPECT_FALSE(state.is_spawned);
  EXPECT_EQ(state.time_of_validity, after_despawn);
  EXPECT_EQ(state.id, actor.id());
  EXPECT_EQ(actor.current_time(), after_despawn);
}

TEST(TrajectoryActorTest, TestGeometry) {
  // SETUP
  const transforms::Frame<3> actor_frame{transforms::Frame<3>::new_frame()};
  curves::TCurve<transforms::FSE3> curve{
      curves::testing::make_circle_curve(simulator::SCENE_FRAME, actor_frame)};

  constexpr time::Timestamp START_TIME{std::chrono::seconds(3)};

  const state::Trajectory trajectory{curve, START_TIME};
  const ActorId id{ActorId::new_uuid()};
  TrajectoryActor actor{id, trajectory};
  actor.simulate_forward(START_TIME);

  // ACTION
  const Geometry geometry = actor.geometry();

  // VERIFICATION
  EXPECT_TRUE(
      std::holds_alternative<Geometry::NoUpdate>(geometry.visible_geometry));
  EXPECT_EQ(geometry.frame, trajectory.body_frame());
  EXPECT_EQ(geometry.time_of_validity, START_TIME);
}

}  // namespace resim::actor
