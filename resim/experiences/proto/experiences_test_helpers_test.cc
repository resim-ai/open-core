// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/experiences/proto/experiences_test_helpers.hh"

#include <gtest/gtest.h>

#include "resim/curves/test_helpers.hh"
#include "resim/curves/two_jet.hh"
#include "resim/experiences/actor.hh"
#include "resim/experiences/completion_criteria.hh"
#include "resim/experiences/dynamic_behavior.hh"
#include "resim/experiences/experience.hh"
#include "resim/experiences/geometry.hh"
#include "resim/experiences/ilqr_drone.hh"
#include "resim/experiences/location_condition.hh"
#include "resim/experiences/storyboard.hh"
#include "resim/geometry/wireframe.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/uuid.hh"

namespace resim::experiences {

namespace {
constexpr time::Timestamp ZERO_TIME;
constexpr time::Timestamp NON_ZERO_TIME{std::chrono::seconds(1)};
constexpr double DIFFERENT_NUMBER = 42.0;
}  // namespace

TEST(ExperienceTestHelpersTest, TestActorEquality) {
  // SETUP/ACTION
  const Actor test_actor_1 = make_test_actor();
  const Actor test_actor_2 = make_test_actor();
  // VERIFICATION
  EXPECT_TRUE(test_actor_equality(test_actor_1, test_actor_1));
  EXPECT_TRUE(test_actor_equality(test_actor_2, test_actor_2));
  EXPECT_FALSE(test_actor_equality(test_actor_1, test_actor_2));
  // Branch tests
  Actor test_actor_3{test_actor_1};
  test_actor_3.name = "different_name";
  EXPECT_FALSE(test_actor_equality(test_actor_1, test_actor_3));
  Actor test_actor_4{test_actor_1};
  test_actor_4.actor_type = ActorType::SYSTEM_UNDER_TEST;
  EXPECT_FALSE(test_actor_equality(test_actor_1, test_actor_4));
  Actor test_actor_5{test_actor_1};
  test_actor_5.geometries = {};
  EXPECT_FALSE(test_actor_equality(test_actor_1, test_actor_5));
}

TEST(ExperienceTestHelpersTest, TestLocationConditionEquality) {
  // SETUP/ACTION
  const LocationCondition test_location_condition_1 =
      make_test_location_condition();
  const LocationCondition test_location_condition_2 =
      make_test_location_condition();
  // VERIFICATION
  EXPECT_TRUE(test_location_condition_equality(
      test_location_condition_1,
      test_location_condition_1));
  EXPECT_TRUE(test_location_condition_equality(
      test_location_condition_2,
      test_location_condition_2));
  EXPECT_FALSE(test_location_condition_equality(
      test_location_condition_1,
      test_location_condition_2));
  // Branch tests
  LocationCondition test_location_condition_3{test_location_condition_1};
  transforms::SE3 different_position = transforms::SE3::identity();
  different_position.set_frames(
      transforms::Frame<3>::new_frame(),
      transforms::Frame<3>::new_frame());
  test_location_condition_3.target_position = different_position;
  EXPECT_FALSE(test_location_condition_equality(
      test_location_condition_1,
      test_location_condition_3));
  LocationCondition test_location_condition_4{test_location_condition_1};
  test_location_condition_4.tolerance_m = DIFFERENT_NUMBER;
  EXPECT_FALSE(test_location_condition_equality(
      test_location_condition_1,
      test_location_condition_4));
}

TEST(ExperienceTestHelpersTest, TestILQRDroneEquality) {
  // SETUP/ACTION
  const ILQRDrone test_drone_1 = make_test_ilqr_drone();
  const ILQRDrone test_drone_2 = make_test_ilqr_drone();
  // VERIFICATION
  EXPECT_TRUE(test_ilqr_drone_equality(test_drone_1, test_drone_1));
  EXPECT_TRUE(test_ilqr_drone_equality(test_drone_2, test_drone_2));
  EXPECT_TRUE(test_ilqr_drone_equality(test_drone_1, test_drone_2));
}

TEST(ExperienceTestHelpersTest, TestCompletionCriteriaEquality) {
  // SETUP/ACTION
  const CompletionCriteria test_completion_criteria_1 =
      make_test_completion_criteria();
  const CompletionCriteria test_completion_criteria_2 =
      make_test_completion_criteria();
  // VERIFICATION
  EXPECT_TRUE(test_completion_criteria_equality(
      test_completion_criteria_1,
      test_completion_criteria_1));
  EXPECT_TRUE(test_completion_criteria_equality(
      test_completion_criteria_2,
      test_completion_criteria_2));
  EXPECT_FALSE(test_completion_criteria_equality(
      test_completion_criteria_1,
      test_completion_criteria_2));
  // Test with different number of conditions
  CompletionCriteria test_completion_criteria_3{test_completion_criteria_1};
  const LocationCondition new_location_condition =
      make_test_location_condition();
  const Condition new_condition{
      .condition = new_location_condition,
      .delay = time::as_duration(0.0)};
  test_completion_criteria_3.conditions.push_back(new_condition);
  EXPECT_FALSE(test_completion_criteria_equality(
      test_completion_criteria_1,
      test_completion_criteria_3));
  // Test branches
  CompletionCriteria test_completion_criteria_4{test_completion_criteria_1};
  test_completion_criteria_4.conditions[0].delay =
      time::as_duration(DIFFERENT_NUMBER);
  EXPECT_FALSE(test_completion_criteria_equality(
      test_completion_criteria_1,
      test_completion_criteria_4));
  CompletionCriteria test_completion_criteria_5{test_completion_criteria_1};
  test_completion_criteria_5.time_limit = time::as_duration(DIFFERENT_NUMBER);
  EXPECT_FALSE(test_completion_criteria_equality(
      test_completion_criteria_1,
      test_completion_criteria_5));
}

TEST(ExperienceTestHelpersTest, TestTrajectoryEquality) {
  // SETUP/ACTION
  curves::TCurve<transforms::SE3> test_curve =
      curves::testing::make_circle_curve();
  const actor::state::Trajectory test_trajectory_1{test_curve, ZERO_TIME};
  const actor::state::Trajectory test_trajectory_2{
      test_curve,
      ZERO_TIME + time::as_duration(1.0)};
  // VERIFICATION
  EXPECT_TRUE(test_trajectory_equality(test_trajectory_1, test_trajectory_1));
  EXPECT_TRUE(test_trajectory_equality(test_trajectory_2, test_trajectory_2));
  EXPECT_FALSE(test_trajectory_equality(test_trajectory_1, test_trajectory_2));
  curves::TCurve<transforms::SE3> empty_curve;
  const actor::state::Trajectory test_trajectory_3{empty_curve, ZERO_TIME};
  EXPECT_FALSE(test_trajectory_equality(test_trajectory_1, test_trajectory_3));
  actor::state::Trajectory test_trajectory_4{test_curve, NON_ZERO_TIME};
  EXPECT_FALSE(test_trajectory_equality(test_trajectory_1, test_trajectory_4));
  curves::TCurve<transforms::SE3> empty_curve_2;
  empty_curve.append(curves::TCurve<transforms::SE3>::Control{
      0.0,
      curves::TwoJetL<transforms::SE3>()});
  empty_curve_2.append(curves::TCurve<transforms::SE3>::Control{
      DIFFERENT_NUMBER,
      curves::TwoJetL<transforms::SE3>()});
  actor::state::Trajectory test_trajectory_5{empty_curve, NON_ZERO_TIME};
  actor::state::Trajectory test_trajectory_6{empty_curve_2, NON_ZERO_TIME};
  EXPECT_FALSE(test_trajectory_equality(test_trajectory_5, test_trajectory_6));
}

TEST(ExperienceTestHelpersTest, TestStoryboardEquality) {
  // SETUP/ACTION
  const Storyboard test_storyboard_1 = make_test_storyboard();
  const Storyboard test_storyboard_2 = make_test_storyboard();
  // VERIFICATION
  EXPECT_TRUE(test_storyboard_equality(test_storyboard_1, test_storyboard_1));
  EXPECT_TRUE(test_storyboard_equality(test_storyboard_2, test_storyboard_2));
  EXPECT_FALSE(test_storyboard_equality(test_storyboard_1, test_storyboard_2));
  // Test with a different number of points
  Storyboard test_storyboard_3{test_storyboard_1};
  test_storyboard_3.movement_models.push_back(
      make_test_trajectory_movement_model());
  EXPECT_FALSE(test_storyboard_equality(test_storyboard_1, test_storyboard_3));
}

TEST(ExperienceTestHelpersTest, TestDynamicBehaviorEquality) {
  // SETUP/ACTION
  const DynamicBehavior test_dynamic_behavior_1 = make_test_dynamic_behavior();
  const DynamicBehavior test_dynamic_behavior_2 = make_test_dynamic_behavior();
  // VERIFICATION
  EXPECT_TRUE(test_dynamic_behavior_equality(
      test_dynamic_behavior_1,
      test_dynamic_behavior_1));
  EXPECT_TRUE(test_dynamic_behavior_equality(
      test_dynamic_behavior_2,
      test_dynamic_behavior_2));
  EXPECT_FALSE(test_dynamic_behavior_equality(
      test_dynamic_behavior_1,
      test_dynamic_behavior_2));
  // Test with a different number of actors
  DynamicBehavior test_dynamic_behavior_3{test_dynamic_behavior_1};
  test_dynamic_behavior_3.actors.push_back(make_test_actor());
  EXPECT_FALSE(test_dynamic_behavior_equality(
      test_dynamic_behavior_1,
      test_dynamic_behavior_3));
  // Branch tests
  DynamicBehavior test_dynamic_behavior_4{test_dynamic_behavior_1};
  test_dynamic_behavior_4.storyboard = make_test_storyboard();
  EXPECT_FALSE(test_dynamic_behavior_equality(
      test_dynamic_behavior_1,
      test_dynamic_behavior_4));
  DynamicBehavior test_dynamic_behavior_5{test_dynamic_behavior_1};
  test_dynamic_behavior_5.completion_criteria = make_test_completion_criteria();
  EXPECT_FALSE(test_dynamic_behavior_equality(
      test_dynamic_behavior_1,
      test_dynamic_behavior_5));
}

TEST(ExperienceTestHelpersTest, TestGeometryEquality) {
  // SETUP/ACTION
  const Geometry geometry_1 = make_test_geometry();
  const Geometry geometry_2 = make_test_geometry();
  // VERIFICATION
  EXPECT_TRUE(test_geometry_equality(geometry_1, geometry_1));
  EXPECT_TRUE(test_geometry_equality(geometry_2, geometry_2));
  EXPECT_FALSE(test_geometry_equality(geometry_1, geometry_2));
}

TEST(ExperienceTestHelpersTest, TestExperienceEquality) {
  // SETUP/ACTION
  const Experience test_experience_1 = make_test_experience();
  const Experience test_experience_2 = make_test_experience();
  // VERIFICATION
  EXPECT_TRUE(test_experience_equality(test_experience_1, test_experience_1));
  EXPECT_TRUE(test_experience_equality(test_experience_2, test_experience_2));
  EXPECT_FALSE(test_experience_equality(test_experience_1, test_experience_2));
  // Branch coverage
  Experience test_experience_3{test_experience_1};
  test_experience_3.header.name = "different name";
  EXPECT_FALSE(test_experience_equality(test_experience_1, test_experience_3));
  Experience test_experience_4{test_experience_1};
  test_experience_4.header.description = "different description";
  EXPECT_FALSE(test_experience_equality(test_experience_1, test_experience_4));
  Experience test_experience_5{test_experience_1};
  test_experience_5.header.parent_experience_name = "different parent";
  EXPECT_FALSE(test_experience_equality(test_experience_1, test_experience_5));
  Experience test_experience_6{test_experience_1};
  test_experience_6.header.revision.major = 2;
  EXPECT_FALSE(test_experience_equality(test_experience_1, test_experience_6));
  Experience test_experience_7{test_experience_1};
  test_experience_7.header.revision.minor = 1;
  EXPECT_FALSE(test_experience_equality(test_experience_1, test_experience_7));
  Experience test_experience_8{test_experience_1};
  test_experience_8.geometries = {};
  EXPECT_FALSE(test_experience_equality(test_experience_1, test_experience_8));
  Experience test_experience_9{test_experience_1};
  const UUID new_geometry_id{UUID::new_uuid()};
  test_experience_9.geometries = {
      {new_geometry_id, test_experience_1.geometries.cbegin()->second}};
  test_experience_9.geometries.begin()->second.id = new_geometry_id;
  EXPECT_FALSE(test_experience_equality(test_experience_1, test_experience_9));
  Experience test_experience_10{test_experience_1};
  test_experience_10.geometries.begin()->second.model = geometry::Wireframe();
  EXPECT_FALSE(test_experience_equality(test_experience_1, test_experience_10));
}

TEST(ExperienceTestHelpersTest, TestMovementModelEquality) {
  // SETUP/ACTION
  const MovementModel movement_model_1 = make_test_trajectory_movement_model();
  MovementModel movement_model_2 = make_test_ilqr_movement_model();
  movement_model_2.actor_reference = movement_model_1.actor_reference;
  // VERIFICATION
  // We expect to test all branches of the match.
  EXPECT_FALSE(
      test_movement_model_equality(movement_model_1, movement_model_2));
  EXPECT_FALSE(
      test_movement_model_equality(movement_model_2, movement_model_1));
}

}  // namespace resim::experiences
