
#include "resim_core/actor/factory.hh"

#include <gtest/gtest.h>

#include <unordered_map>
#include <variant>
#include <vector>

#include "resim_core/actor/actor.hh"
#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/curves/test_helpers.hh"
#include "resim_core/experiences/actor.hh"
#include "resim_core/experiences/dynamic_behavior.hh"
#include "resim_core/experiences/geometry.hh"
#include "resim_core/experiences/ilqr_drone.hh"
#include "resim_core/experiences/storyboard.hh"
#include "resim_core/geometry/drone_wireframe.hh"
#include "resim_core/geometry/wireframe.hh"
#include "resim_core/simulator/standard_frames.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/uuid.hh"

namespace resim::actor {
namespace {

constexpr time::Timestamp ZERO_TIME;

// Simple helper to make a trajectory
state::Trajectory trajectory_from_curve(
    const curves::TCurve<transforms::SE3> &curve) {
  constexpr time::Timestamp ZERO_TIME;
  return state::Trajectory{curve, ZERO_TIME};
}

// Helper to ensure that a given actor follows a given curve
void expect_actor_matches_curve(
    const curves::TCurve<transforms::SE3> &curve,
    Actor &actor) {
  constexpr int NUM_POINTS = 10;
  for (int ii = 0; ii < NUM_POINTS; ++ii) {
    const double frac = static_cast<double>(ii) / (NUM_POINTS - 1);

    double t_s = frac * curve.end_time() + (1. - frac) * curve.start_time();
    time::Timestamp time{time::as_duration(t_s)};

    // doubles are more accurate than Timestamps for small times, so we need
    // to clamp this to a value representable by time::Timestamp for the poses
    // to match.
    t_s = time::as_seconds(time.time_since_epoch());

    actor.simulate_forward(time::Timestamp{time::as_duration(t_s)});

    EXPECT_TRUE(
        actor.observable_state().state.ref_from_body_two_jet().is_approx(
            curve.point_at(t_s).right_two_jet()));
  }
}
}  // namespace

TEST(FactoryTest, TestMakeTrajectoryActor) {
  // SETUP
  experiences::DynamicBehavior dynamic_behavior;
  const ActorId id{ActorId::new_uuid()};
  const transforms::Frame<3> frame{transforms::Frame<3>::new_frame()};
  const curves::TCurve<transforms::SE3> t_curve{
      curves::testing::make_circle_curve(frame, simulator::SCENE_FRAME)};
  dynamic_behavior.actors.push_back(experiences::Actor{
      .id = id,
      .actor_type = experiences::ActorType::SIMULATION_ACTOR,
  });
  dynamic_behavior.storyboard.movement_models.push_back(
      experiences::MovementModel{
          .actor_reference = id,
          .model = trajectory_from_curve(t_curve),
      });

  // ACTION
  const std::vector<std::unique_ptr<Actor>> actors{
      factory(dynamic_behavior, {})};

  // VERIFICATION
  ASSERT_EQ(actors.size(), 1U);
  ASSERT_NE(actors.front(), nullptr);
  expect_actor_matches_curve(t_curve, *actors.front());
  EXPECT_EQ(actors.front()->id(), id);
}

TEST(FactoryTest, TestFailsOnBadMovementModel) {
  // SETUP
  experiences::DynamicBehavior dynamic_behavior;
  const ActorId id{ActorId::new_uuid()};
  const transforms::Frame<3> frame{transforms::Frame<3>::new_frame()};
  const curves::TCurve<transforms::SE3> t_curve{
      curves::testing::make_circle_curve(frame, simulator::SCENE_FRAME)};
  dynamic_behavior.actors.push_back(experiences::Actor{
      .id = id,
      .actor_type = experiences::ActorType::SIMULATION_ACTOR,
  });

  // Not enough models:

  // ACTION / VERIFICATION
  EXPECT_THROW(factory(dynamic_behavior, {}), AssertException);

  // SETUP
  // Unsupported model:
  dynamic_behavior.storyboard.movement_models.push_back(
      experiences::MovementModel{
          .actor_reference = id,
          .model = experiences::ILQRDrone{},
      });
  // ACTION / VERIFICATION
  EXPECT_THROW(factory(dynamic_behavior, {}), AssertException);

  // SETUP
  // Multiple models:
  dynamic_behavior.storyboard.movement_models.clear();
  for (int ii = 0; ii < 2; ++ii) {
    dynamic_behavior.storyboard.movement_models.push_back(
        experiences::MovementModel{
            .actor_reference = id,
            .model = trajectory_from_curve(t_curve),
        });
  }
  // ACTION / VERIFICATION
  EXPECT_THROW(factory(dynamic_behavior, {}), AssertException);
}

TEST(FactoryTest, TestFailsOnBadActorType) {
  // SETUP
  experiences::DynamicBehavior dynamic_behavior;
  const ActorId id{ActorId::new_uuid()};
  const transforms::Frame<3> frame{transforms::Frame<3>::new_frame()};
  const curves::TCurve<transforms::SE3> t_curve{
      curves::testing::make_circle_curve(frame, simulator::SCENE_FRAME)};
  dynamic_behavior.actors.push_back(experiences::Actor{
      .id = id,
      .actor_type = experiences::ActorType::INVALID,  // <- Invalid actor type
  });
  dynamic_behavior.storyboard.movement_models.push_back(
      experiences::MovementModel{
          .actor_reference = id,
          .model = trajectory_from_curve(t_curve),
      });

  // ACTION / VERIFICATION
  EXPECT_THROW(factory(dynamic_behavior, {}), AssertException);
}

TEST(FactoryTest, TestForwardsGeometry) {
  // SETUP
  experiences::DynamicBehavior dynamic_behavior;
  const ActorId id{ActorId::new_uuid()};
  const UUID geometry_id{UUID::new_uuid()};
  const transforms::Frame<3> frame{transforms::Frame<3>::new_frame()};
  const curves::TCurve<transforms::SE3> t_curve{
      curves::testing::make_circle_curve(frame, simulator::SCENE_FRAME)};
  dynamic_behavior.actors.push_back(experiences::Actor{
      .id = id,
      .actor_type = experiences::ActorType::SIMULATION_ACTOR,
      .geometries = {{
          .geometry_id = geometry_id,
      }},
  });
  dynamic_behavior.storyboard.movement_models.push_back(
      experiences::MovementModel{
          .actor_reference = id,
          .model = trajectory_from_curve(t_curve),
      });

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
  const geometry::Wireframe wireframe{geometry::drone_wireframe(extents)};

  const std::unordered_map<UUID, experiences::Geometry> geometries{
      {geometry_id,
       {
           .id = geometry_id,
           .model = wireframe,
       }},
  };

  // ACTION
  const std::vector<std::unique_ptr<Actor>> actors{
      factory(dynamic_behavior, geometries)};

  ASSERT_EQ(actors.size(), 1U);
  const auto &actor = actors.front();
  actor->simulate_forward(ZERO_TIME);

  // VERIFICATION
  const Geometry &geometry = actor->geometry();
  ASSERT_TRUE(
      std::holds_alternative<geometry::Wireframe>(geometry.visible_geometry));
  EXPECT_EQ(
      std::get<geometry::Wireframe>(geometry.visible_geometry),
      wireframe);
}

TEST(FactoryTest, TestBadGeometry) {
  // SETUP
  experiences::DynamicBehavior dynamic_behavior;
  const ActorId id{ActorId::new_uuid()};
  const UUID geometry_id{UUID::new_uuid()};
  const transforms::Frame<3> frame{transforms::Frame<3>::new_frame()};
  const curves::TCurve<transforms::SE3> t_curve{
      curves::testing::make_circle_curve(frame, simulator::SCENE_FRAME)};
  dynamic_behavior.actors.push_back(experiences::Actor{
      .id = id,
      .actor_type = experiences::ActorType::SIMULATION_ACTOR,
      .geometries = {{
          .geometry_id = geometry_id,
      }},
  });
  dynamic_behavior.storyboard.movement_models.push_back(
      experiences::MovementModel{
          .actor_reference = id,
          .model = trajectory_from_curve(t_curve),
      });
  std::unordered_map<UUID, experiences::Geometry> geometries{};

  // ACTION / VERIFICATION
  // Should be unable to find the needed geometry
  EXPECT_THROW(factory(dynamic_behavior, geometries), AssertException);
}

TEST(FactoryTest, TestMultipleGeometries) {
  // SETUP
  experiences::DynamicBehavior dynamic_behavior;
  const ActorId id{ActorId::new_uuid()};
  const transforms::Frame<3> frame{transforms::Frame<3>::new_frame()};
  const curves::TCurve<transforms::SE3> t_curve{
      curves::testing::make_circle_curve(frame, simulator::SCENE_FRAME)};

  const UUID geometry_id{UUID::new_uuid()};
  const UUID other_geometry_id{UUID::new_uuid()};
  // Add multiple geometries for a single actor which isn't yet supported
  dynamic_behavior.actors.push_back(experiences::Actor{
      .id = id,
      .actor_type = experiences::ActorType::SIMULATION_ACTOR,
      .geometries =
          {
              {
                  .geometry_id = geometry_id,
              },
              {
                  .geometry_id = other_geometry_id,
              },
          },
  });
  std::unordered_map<UUID, experiences::Geometry> geometries{};
  geometries.emplace(
      geometry_id,
      experiences::Geometry{
          .id = geometry_id,
          .model = geometry::Wireframe(),
      });
  geometries.emplace(
      other_geometry_id,
      experiences::Geometry{
          .id = other_geometry_id,
          .model = geometry::Wireframe(),
      });

  // ACTION / VERIFICATION
  EXPECT_THROW(factory(dynamic_behavior, geometries), AssertException);
}

}  // namespace resim::actor
