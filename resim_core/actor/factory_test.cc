
#include "resim_core/actor/factory.hh"

#include <gtest/gtest.h>

#include <vector>

#include "resim_core/actor/actor.hh"
#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/curves/test_helpers.hh"
#include "resim_core/experiences/actor.hh"
#include "resim_core/experiences/dynamic_behavior.hh"
#include "resim_core/experiences/ilqr_drone.hh"
#include "resim_core/experiences/storyboard.hh"
#include "resim_core/simulator/standard_frames.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"

namespace resim::actor {
namespace {

// Simple helper to make a trajectory
state::Trajectory trajectory_from_curve(
    const curves::TCurve<transforms::FSE3> &curve) {
  constexpr time::Timestamp ZERO_TIME;
  return state::Trajectory{curve, ZERO_TIME};
}

// Helper to ensure that a given actor follows a given curve
void expect_actor_matches_curve(
    const curves::TCurve<transforms::FSE3> &curve,
    Actor &actor) {
  constexpr int NUM_POINTS = 10;
  for (int ii = 0; ii < NUM_POINTS; ++ii) {
    const double frac = static_cast<double>(ii) / (NUM_POINTS - 1);

    double t_s = frac * curve.end_time() + (1. - frac) * curve.start_time();
    time::Timestamp time{time::as_duration(t_s)};

    // doubles are more accurate than Timestamps for small times, so we need to
    // clamp this to a value representable by time::Timestamp for the poses to
    // match.
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
  const curves::TCurve<transforms::FSE3> t_curve{
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
  const std::vector<std::unique_ptr<Actor>> actors{factory(dynamic_behavior)};

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
  const curves::TCurve<transforms::FSE3> t_curve{
      curves::testing::make_circle_curve(frame, simulator::SCENE_FRAME)};
  dynamic_behavior.actors.push_back(experiences::Actor{
      .id = id,
      .actor_type = experiences::ActorType::SIMULATION_ACTOR,
  });

  // Not enough models:

  // ACTION / VERIFICATION
  EXPECT_THROW(factory(dynamic_behavior), AssertException);

  // SETUP
  // Unsupported model:
  dynamic_behavior.storyboard.movement_models.push_back(
      experiences::MovementModel{
          .actor_reference = id,
          .model = experiences::ILQRDrone{},
      });
  // ACTION / VERIFICATION
  EXPECT_THROW(factory(dynamic_behavior), AssertException);

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
  EXPECT_THROW(factory(dynamic_behavior), AssertException);
}

TEST(FactoryTest, TestFailsOnBadActorType) {
  // SETUP
  experiences::DynamicBehavior dynamic_behavior;
  const ActorId id{ActorId::new_uuid()};
  const transforms::Frame<3> frame{transforms::Frame<3>::new_frame()};
  const curves::TCurve<transforms::FSE3> t_curve{
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
  EXPECT_THROW(factory(dynamic_behavior), AssertException);
}

}  // namespace resim::actor
