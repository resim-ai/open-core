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
#include "resim_core/experiences/geometry.hh"
#include "resim_core/geometry/drone_wireframe.hh"
#include "resim_core/geometry/wireframe.hh"
#include "resim_core/simulator/standard_frames.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::actor {

namespace {

using Frame = transforms::Frame<transforms::SE3::DIMS>;
constexpr double CHASSIS_RADIUS_M = 1.;
constexpr double ROTOR_LATERAL_OFFSET_M = 0.3;
constexpr double ROTOR_VERTICAL_OFFSET_M = 0.3;
constexpr double ROTOR_RADIUS_M = 0.5;
constexpr std::size_t SAMPLES_PER_ROTOR = 2;

const geometry::DroneExtents extents{
    .chassis_radius_m = CHASSIS_RADIUS_M,
    .rotor_lateral_offset_m = ROTOR_LATERAL_OFFSET_M,
    .rotor_vertical_offset_m = ROTOR_VERTICAL_OFFSET_M,
    .rotor_radius_m = ROTOR_RADIUS_M,
    .samples_per_rotor = SAMPLES_PER_ROTOR,
};

}  // namespace

TEST(TrajectoryActorTest, TestConstruction) {
  // SETUP
  const Frame actor_frame{Frame::new_frame()};
  const curves::TCurve<transforms::SE3> curve{
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
  const Frame actor_frame{Frame::new_frame()};
  const curves::TCurve<transforms::SE3> curve{
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
  const Frame actor_frame{Frame::new_frame()};
  const curves::TCurve<transforms::SE3> curve{
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
  const Frame actor_frame{Frame::new_frame()};
  const curves::TCurve<transforms::SE3> curve{
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
  const Frame actor_frame{Frame::new_frame()};
  curves::TCurve<transforms::SE3> curve{
      curves::testing::make_circle_curve(simulator::SCENE_FRAME, actor_frame)};

  constexpr time::Timestamp START_TIME{std::chrono::seconds(3)};

  const state::Trajectory trajectory{curve, START_TIME};
  const ActorId id{ActorId::new_uuid()};

  const time::Timestamp end_time = trajectory.end_time();

  const geometry::Wireframe wireframe{geometry::drone_wireframe(extents)};
  const experiences::Geometry geometry{
      .id = UUID::new_uuid(),
      .model = wireframe};

  TrajectoryActor actor{id, trajectory, geometry};

  constexpr auto EPSILON_TIME = std::chrono::nanoseconds(1);
  {
    // Before start of the trajectory (when actor spawns),
    // no geometry should be published.
    // ACTION
    actor.simulate_forward(START_TIME - EPSILON_TIME);
    const Geometry geometry = actor.geometry();

    // VERIFICATION
    EXPECT_TRUE(
        std::holds_alternative<Geometry::NoUpdate>(geometry.visible_geometry));
    EXPECT_EQ(geometry.frame, trajectory.body_frame());
    EXPECT_EQ(geometry.time_of_validity, START_TIME - EPSILON_TIME);
  }
  {
    // After spawning the actor, geometry should be published once.
    // ACTION
    actor.simulate_forward(START_TIME);
    const Geometry geometry = actor.geometry();

    // VERIFICATION
    EXPECT_TRUE(
        std::holds_alternative<geometry::Wireframe>(geometry.visible_geometry));
    const geometry::Wireframe &result_wireframe{
        std::get<geometry::Wireframe>(geometry.visible_geometry)};
    EXPECT_EQ(result_wireframe.points(), wireframe.points());
    EXPECT_EQ(result_wireframe.edges(), wireframe.edges());
    EXPECT_EQ(geometry.frame, trajectory.body_frame());
    EXPECT_EQ(geometry.time_of_validity, START_TIME);
  }
  {
    // At end (and intermediate times), no update should be published).
    // ACTION
    actor.simulate_forward(end_time);
    const Geometry geometry = actor.geometry();

    // VERIFICATION
    EXPECT_TRUE(
        std::holds_alternative<Geometry::NoUpdate>(geometry.visible_geometry));
    EXPECT_EQ(geometry.frame, trajectory.body_frame());
    EXPECT_EQ(geometry.time_of_validity, end_time);
  }
  {
    // After the end, geometry should be cleared
    // ACTION
    actor.simulate_forward(end_time + EPSILON_TIME);
    const Geometry geometry = actor.geometry();

    // VERIFICATION
    EXPECT_TRUE(
        std::holds_alternative<Geometry::Clear>(geometry.visible_geometry));
    EXPECT_EQ(geometry.frame, trajectory.body_frame());
    EXPECT_EQ(geometry.time_of_validity, end_time + EPSILON_TIME);
  }
  {
    // After the final clear, geometry should not update.
    // ACTION
    actor.simulate_forward(end_time + 2 * EPSILON_TIME);
    const Geometry geometry = actor.geometry();

    // VERIFICATION
    EXPECT_TRUE(
        std::holds_alternative<Geometry::NoUpdate>(geometry.visible_geometry));
    EXPECT_EQ(geometry.frame, trajectory.body_frame());
    EXPECT_EQ(geometry.time_of_validity, end_time + 2 * EPSILON_TIME);
  }
}

}  // namespace resim::actor
