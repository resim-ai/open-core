// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/experiences/proto/experiences_test_helpers.hh"

#include <algorithm>
#include <unordered_map>
#include <variant>
#include <vector>

#include "resim/actor/state/trajectory.hh"
#include "resim/curves/t_curve.hh"
#include "resim/curves/test_helpers.hh"
#include "resim/experiences/actor.hh"
#include "resim/experiences/completion_criteria.hh"
#include "resim/experiences/geometry.hh"
#include "resim/experiences/ilqr_drone.hh"
#include "resim/experiences/location_condition.hh"
#include "resim/experiences/storyboard.hh"
#include "resim/geometry/drone_wireframe.hh"
#include "resim/geometry/wireframe.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/match.hh"
#include "resim/utils/uuid.hh"

namespace resim::experiences {

namespace {
constexpr auto TEST_ACTOR_NAME = "test_actor";
constexpr double TEST_TOLERANCE{2.0};
constexpr double ONE_MINUTE{
    std::chrono::seconds(std::chrono::minutes(1)).count()};
constexpr time::Timestamp ZERO_TIME;
}  // namespace

Actor make_test_actor(ActorType actor_type) {
  return {
      .id = UUID::new_uuid(),
      .name = TEST_ACTOR_NAME,
      .actor_type = actor_type,
      .geometries = {{
          .geometry_id = UUID::new_uuid(),
      }},
  };
}

bool test_actor_equality(
    const Actor& expected_actor,
    const Actor& actual_actor) {
  return expected_actor.id == actual_actor.id &&
         expected_actor.name == actual_actor.name &&
         expected_actor.actor_type == actual_actor.actor_type &&
         expected_actor.geometries == actual_actor.geometries;
}

LocationCondition make_test_location_condition() {
  return {
      .triggering_actor = UUID::new_uuid(),
      .target_position = transforms::SE3::identity(),
      .tolerance_m = TEST_TOLERANCE,
  };
}

bool test_location_condition_equality(
    const LocationCondition& expected_location_condition,
    const LocationCondition& actual_location_condition) {
  return expected_location_condition.triggering_actor ==
             actual_location_condition.triggering_actor &&
         expected_location_condition.target_position.is_approx(
             actual_location_condition.target_position) &&
         expected_location_condition.tolerance_m ==
             actual_location_condition.tolerance_m;
}

CompletionCriteria make_test_completion_criteria() {
  LocationCondition location_condition_1 = make_test_location_condition();
  LocationCondition location_condition_2 = make_test_location_condition();
  Condition condition_1{
      .condition = location_condition_1,
      .delay = time::as_duration(0.0),  // zero second delay
  };
  Condition condition_2{
      .condition = location_condition_2,
      .delay = time::as_duration(1.0),  // one second delay
  };
  return {
      .time_limit = time::as_duration(ONE_MINUTE),  // one minute
      .conditions = {condition_1, condition_2},
  };
}

bool test_conditions_equal(
    const std::vector<Condition>& expected_conditions,
    const std::vector<Condition>& actual_conditions) {
  if (expected_conditions.size() != actual_conditions.size()) {
    return false;
  }
  bool condition_equal = true;
  for (int i = 0; i < expected_conditions.size(); i++) {
    condition_equal &=
        test_location_condition_equality(
            expected_conditions.at(i).condition,
            actual_conditions.at(i).condition) &&
        expected_conditions.at(i).delay == actual_conditions.at(i).delay;
  }
  return condition_equal;
}

bool test_completion_criteria_equality(
    const CompletionCriteria& expected_completion_criteria,
    const CompletionCriteria& actual_completion_criteria) {
  return test_conditions_equal(
             expected_completion_criteria.conditions,
             actual_completion_criteria.conditions) &&
         expected_completion_criteria.time_limit ==
             actual_completion_criteria.time_limit;
}

ILQRDrone make_test_ilqr_drone() {
  return ILQRDrone{
      .velocity_cost = 82.0,
      .initial_position = Eigen::Vector3d{4.0, 5.0, 6.0},
      .goal_position = Eigen::Vector3d{1.0, 2.0, 3.0},
  };
}

bool test_ilqr_drone_equality(
    const ILQRDrone& expected_ilqr_drone,
    const ILQRDrone& actual_ilqr_drone) {
  return true;
}

MovementModel make_test_ilqr_movement_model() {
  return {.actor_reference = UUID::new_uuid(), .model = make_test_ilqr_drone()};
}

MovementModel make_test_trajectory_movement_model() {
  curves::TCurve<transforms::SE3> test_curve =
      curves::testing::make_circle_curve();

  actor::state::Trajectory test_trajectory{test_curve, ZERO_TIME};
  return {
      .actor_reference = UUID::new_uuid(),
      .model = test_trajectory,
  };
}

bool test_trajectory_equality(
    const actor::state::Trajectory& expected_trajectory,
    const actor::state::Trajectory& actual_trajectory) {
  const std::vector<typename curves::TCurve<transforms::SE3>::Control>&
      expected_points = expected_trajectory.curve().control_pts();
  const std::vector<typename curves::TCurve<transforms::SE3>::Control>&
      actual_points = actual_trajectory.curve().control_pts();
  if (expected_points.size() != actual_points.size()) {
    return false;
  }
  bool trajectory_equal = true;
  for (unsigned int i = 0; i < expected_points.size(); ++i) {
    trajectory_equal &= expected_points[i].time == actual_points[i].time;
    const curves::TwoJetL<transforms::SE3>& expected_two_jet =
        expected_points[i].point;
    const curves::TwoJetL<transforms::SE3>& actual_two_jet =
        actual_points[i].point;
    trajectory_equal &= expected_two_jet.is_approx(actual_two_jet);
  }
  return trajectory_equal &&
         expected_trajectory.start_time() == actual_trajectory.start_time();
}

bool test_movement_model_equality(
    const MovementModel& expected_movement_model,
    const MovementModel& actual_movement_model) {
  if (expected_movement_model.actor_reference !=
      actual_movement_model.actor_reference) {
    return false;
  }
  bool movement_model_equal = true;
  match(
      expected_movement_model.model,
      [&](const ILQRDrone& expected_model) {
        match(
            actual_movement_model.model,
            [&](const ILQRDrone& actual_model) {
              movement_model_equal =
                  test_ilqr_drone_equality(expected_model, actual_model);
            },
            [&](const actor::state::Trajectory& actual_trajectory) {
              movement_model_equal = false;
            });
      },
      [&](const actor::state::Trajectory& expected_trajectory) {
        match(
            actual_movement_model.model,
            [&](const ILQRDrone& actual_model) {
              movement_model_equal = false;
            },
            [&](const actor::state::Trajectory& actual_trajectory) {
              movement_model_equal = test_trajectory_equality(
                  expected_trajectory,
                  actual_trajectory);
            });
      });
  return movement_model_equal;
}

Storyboard make_test_storyboard() {
  const MovementModel& movement_model_1 = make_test_ilqr_movement_model();
  const MovementModel& movement_model_2 = make_test_trajectory_movement_model();
  return {.movement_models = {movement_model_1, movement_model_2}};
}

bool test_storyboard_equality(
    const Storyboard& expected_storyboard,
    const Storyboard& actual_storyboard) {
  if (expected_storyboard.movement_models.size() !=
      actual_storyboard.movement_models.size()) {
    return false;
  }
  bool models_equal = true;
  for (int i = 0; i < expected_storyboard.movement_models.size(); i++) {
    models_equal &= test_movement_model_equality(
        expected_storyboard.movement_models.at(i),
        actual_storyboard.movement_models.at(i));
  }
  return models_equal;
}

DynamicBehavior make_test_dynamic_behavior() {
  return {
      .actors = {make_test_actor(), make_test_actor()},
      .storyboard = make_test_storyboard(),
      .completion_criteria = make_test_completion_criteria(),
  };
}

bool test_dynamic_behavior_equality(
    const DynamicBehavior& expected_dynamic_behavior,
    const DynamicBehavior& actual_dynamic_behavior) {
  if (expected_dynamic_behavior.actors.size() !=
      actual_dynamic_behavior.actors.size()) {
    return false;
  }
  bool actors_equal = true;
  for (int i = 0; i < expected_dynamic_behavior.actors.size(); i++) {
    actors_equal &= test_actor_equality(
        expected_dynamic_behavior.actors.at(i),
        actual_dynamic_behavior.actors.at(i));
  }
  return actors_equal &&
         test_storyboard_equality(
             expected_dynamic_behavior.storyboard,
             actual_dynamic_behavior.storyboard) &&
         test_completion_criteria_equality(
             expected_dynamic_behavior.completion_criteria,
             actual_dynamic_behavior.completion_criteria);
}

Revision make_test_revision() { return {.major = 1, .minor = 2}; }

bool test_revision_equality(
    const Revision& expected_revision,
    const Revision& actual_revision) {
  return expected_revision.major == actual_revision.major &&
         expected_revision.minor == actual_revision.minor;
}

Header make_test_header() {
  return {
      .revision = make_test_revision(),
      .name = "test_experience",
      .description = "test_experience_description",
      .parent_experience_name = "parent_experience",
  };
}

bool test_header_equality(
    const Header& expected_header,
    const Header& actual_header) {
  return test_revision_equality(
             expected_header.revision,
             actual_header.revision) &&
         expected_header.name == actual_header.name &&
         expected_header.description == actual_header.description &&
         expected_header.parent_experience_name ==
             actual_header.parent_experience_name;
}

Geometry make_test_geometry() {
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
  return Geometry{
      .id = UUID::new_uuid(),
      .model = geometry::drone_wireframe(extents),
  };
}

bool test_geometry_equality(const Geometry& a, const Geometry& b) {
  if (a.id != b.id) {
    return false;
  }
  return match(a.model, [&](const geometry::Wireframe& wireframe_a) {
    // Only alternative for now so we REASSERT this to please codecov
    REASSERT(std::holds_alternative<geometry::Wireframe>(b.model));
    const geometry::Wireframe& wireframe_b{
        std::get<geometry::Wireframe>(b.model)};
    return wireframe_a == wireframe_b;
  });
}

std::unordered_map<UUID, Geometry> make_test_geometries() {
  const Geometry wireframe_geometry{make_test_geometry()};
  return {{wireframe_geometry.id, wireframe_geometry}};
}

bool test_geometries_equality(
    const std::unordered_map<UUID, Geometry>& a,
    const std::unordered_map<UUID, Geometry>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  return std::all_of(a.cbegin(), a.cend(), [&](const auto& key_and_value) {
    const auto& [k, v] = key_and_value;
    return b.contains(k) and test_geometry_equality(v, b.at(k));
  });
}

Experience make_test_experience() {
  return {
      .header = make_test_header(),
      .dynamic_behavior = make_test_dynamic_behavior(),
      .geometries = make_test_geometries(),
  };
}

bool test_experience_equality(
    const Experience& expected_experience,
    const Experience& actual_experience) {
  return test_header_equality(
             expected_experience.header,
             actual_experience.header) &&
         test_dynamic_behavior_equality(
             expected_experience.dynamic_behavior,
             actual_experience.dynamic_behavior) &&
         test_geometries_equality(
             expected_experience.geometries,
             actual_experience.geometries);
}

}  // namespace resim::experiences
