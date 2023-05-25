#pragma once

#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/experiences/actor.hh"
#include "resim_core/experiences/completion_criteria.hh"
#include "resim_core/experiences/dynamic_behavior.hh"
#include "resim_core/experiences/experience.hh"
#include "resim_core/experiences/geometry.hh"
#include "resim_core/experiences/ilqr_drone.hh"
#include "resim_core/experiences/location_condition.hh"
#include "resim_core/experiences/storyboard.hh"

// A library of makers and equality checkers for experiences.
namespace resim::experiences {

Actor make_test_actor(ActorType actor_type = ActorType::SIMULATION_ACTOR);

bool test_actor_equality(
    const Actor& expected_actor,
    const Actor& actual_actor);

LocationCondition make_test_location_condition();

bool test_location_condition_equality(
    const LocationCondition& expected_location_condition,
    const LocationCondition& actual_location_condition);

CompletionCriteria make_test_completion_criteria();

bool test_completion_criteria_equality(
    const CompletionCriteria& expected_completion_criteria,
    const CompletionCriteria& actual_completion_criteria);

ILQRDrone make_test_ilqr_drone();

bool test_ilqr_drone_equality(
    const ILQRDrone& expected_ilqr_drone,
    const ILQRDrone& actual_ilqr_drone);

MovementModel make_test_ilqr_movement_model();

MovementModel make_test_trajectory_movement_model();

bool test_movement_model_equality(
    const MovementModel& expected_movement_model,
    const MovementModel& actual_movement_model);

Storyboard make_test_storyboard();

bool test_trajectory_equality(
    const actor::state::Trajectory& expected_trajectory,
    const actor::state::Trajectory& actual_trajectory);

bool test_storyboard_equality(
    const Storyboard& expected_storyboard,
    const Storyboard& actual_storyboard);

DynamicBehavior make_test_dynamic_behavior();

bool test_dynamic_behavior_equality(
    const DynamicBehavior& expected_dynamic_behavior,
    const DynamicBehavior& actual_dynamic_behavior);

Geometry make_test_geometry();

bool test_geometry_equality(const Geometry& a, const Geometry& b);

Experience make_test_experience();

bool test_experience_equality(
    const Experience& expected_experience,
    const Experience& actual_experience);

}  // namespace resim::experiences
