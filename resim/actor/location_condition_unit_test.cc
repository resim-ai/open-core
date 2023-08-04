// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/actor/location_condition_unit.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <random>
#include <vector>

#include "resim/actor/actor_id.hh"
#include "resim/actor/state/observable_state.hh"
#include "resim/actor/state/rigid_body_state.hh"
#include "resim/assert/assert.hh"
#include "resim/experiences/completion_criteria.hh"
#include "resim/experiences/location_condition.hh"
#include "resim/simulator/executor_builder.hh"
#include "resim/simulator/standard_topics.hh"
#include "resim/simulator/step_executor.hh"
#include "resim/simulator/time_lord.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/inout.hh"

namespace resim::actor {
namespace {
std::mt19937 rng;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

using experiences::CompletionCriteria;
using experiences::Condition;
using experiences::LocationCondition;
using simulator::ExecutorBuilder;
using simulator::StepExecutor;
using simulator::TimeLord;
using simulator::TimeLordUpdate;

using std::chrono_literals::operator""ms;

const time::Duration NO_DURATION{0ms};
const time::Duration HALF_SECOND{500ms};
const time::Duration THREE_QUARTER_SECOND{750ms};
const time::Duration ONE_SECOND{1000ms};
const time::Duration TWO_SECONDS{2000ms};
const time::Duration TIME_LIMIT{5000ms};
const double DEFAULT_TOL_M = 1e-6;
const double HALF_M = 0.5;

}  // namespace

template <typename Rng>
transforms::SE3 random_se3(Rng &&rng) {
  return transforms::SE3::exp(
      testing::random_vector<typename transforms::SE3::TangentVector>(
          std::forward<Rng>(rng)));
}
const std::vector<actor::ActorId> ACTOR_IDS{UUID::new_uuid(), UUID::new_uuid()};
const std::vector<transforms::SE3> START_POSES{
    random_se3(rng),
    random_se3(rng)};
const std::vector<transforms::SE3> END_POSES{random_se3(rng), random_se3(rng)};

const std::vector<transforms::SE3> MIXED_POSES{START_POSES[0], END_POSES[1]};
const std::vector<transforms::SE3> INCORRECT_END_POSES{
    END_POSES[1],
    END_POSES[0]};

const std::vector<transforms::SE3> END_POSES_OFFSET_BY_ONE_METER{
    transforms::SE3(Eigen::Vector3d(1.0, 0.0, 0.0)) * END_POSES[0],
    transforms::SE3(Eigen::Vector3d(0.0, 1.0, 0.0)) * END_POSES[1],
};

const std::vector<transforms::SE3> END_POSES_OFFSET_BY_POINT_ONE_METERS{
    transforms::SE3(Eigen::Vector3d(0.1, 0.0, 0.0)) * END_POSES[0],
    transforms::SE3(Eigen::Vector3d(0.0, 0.1, 0.0)) * END_POSES[1],
};

void setup_location_condition_test(
    InOut<ExecutorBuilder> executor_builder,
    InOut<TimeLord> time_lord,
    InOut<time::Timestamp> global_time,
    const std::vector<transforms::SE3> &initial_poses,
    const std::vector<transforms::SE3> &final_poses,
    const std::vector<transforms::SE3> &goal_poses,
    const std::vector<actor::ActorId> &ids,
    const std::vector<time::Duration> &delays,
    const double tolerance_m = DEFAULT_TOL_M) {
  REASSERT(initial_poses.size() == final_poses.size());
  REASSERT(initial_poses.size() == goal_poses.size());
  REASSERT(initial_poses.size() == ids.size());

  // Adds completion criteria, computed using aligned goal poses, IDs, and
  // delays.
  std::vector<Condition> conditions{};
  conditions.reserve(goal_poses.size());

  for (int ii = 0; ii < goal_poses.size(); ++ii) {
    conditions.push_back(Condition{
        .condition =
            LocationCondition{
                .triggering_actor = ids[ii],
                .target_position = goal_poses[ii],
                .tolerance_m = tolerance_m,
            },
        .delay = delays[ii]});
  }
  const CompletionCriteria completion_criteria{TIME_LIMIT, conditions};
  LocationConditionUnit location_condition_unit{
      nullptr,
      completion_criteria,
      InOut{executor_builder}};

  // Adds three updates at 0, 1, and 2 seconds. Some subsequence of these will
  // run depending on completion criteria.
  time_lord->schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});
  time_lord->schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(1)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});
  time_lord->schedule_update(TimeLordUpdate{
      time::Timestamp(std::chrono::seconds(2)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE_THEN_TERMINATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED});

  // Start poses are published at t=0s. Otherwise end poses are published.
  for (int ii = 0; ii < initial_poses.size(); ++ii) {
    executor_builder->add_task<time::Timestamp, state::ObservableState>(
        "publish_actor_states",
        simulator::TIME_TOPIC,
        simulator::ACTOR_STATES_TOPIC,
        [&initial_poses, &final_poses, &ids, ii](
            const time::Timestamp current_time) {
          const std::vector<transforms::SE3> &poses{
              (current_time == time::Timestamp(std::chrono::seconds(0)))
                  ? initial_poses
                  : final_poses};

          state::ObservableState state{
              .id = ids[ii],
              .is_spawned = true,
              .time_of_validity = current_time,
              .state = state::RigidBodyState<transforms::SE3>{poses[ii]}};

          return state;
        });
  }

  // Publishes time to referenced time variable, to check correct time.
  executor_builder->add_task<time::Timestamp>(
      "update_time",
      simulator::TIME_TOPIC,
      simulator::NULL_TOPIC,
      [&global_time = *global_time](const time::Timestamp current_time) {
        global_time = current_time;
      });
}

TEST(LocationConditionUnitTest, TestNoDelay) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};
  time::Timestamp time{};

  setup_location_condition_test(
      InOut{executor_builder},
      InOut{time_lord},
      InOut{time},
      START_POSES,
      END_POSES,
      END_POSES,
      ACTOR_IDS,
      std::vector<time::Duration>{NO_DURATION, NO_DURATION});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // TEST + VERIFY

  // Should run exactly two of the updates, skipping the third.
  for (int ii = 0; ii < 2; ++ii) {
    EXPECT_FALSE(time_lord.ready_to_terminate());

    executor->run_step();

    EXPECT_TRUE(time == time::Timestamp(std::chrono::seconds(ii)));
  }

  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(LocationConditionUnitTest, TestFail) {
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};
  time::Timestamp time;

  setup_location_condition_test(
      InOut{executor_builder},
      InOut{time_lord},
      InOut{time},
      START_POSES,
      END_POSES,
      INCORRECT_END_POSES,
      ACTOR_IDS,
      std::vector<time::Duration>{NO_DURATION, NO_DURATION});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // TEST + VERIFY

  // Should run all three updates, as goal is never reached.
  for (int ii = 0; ii < 3; ++ii) {
    EXPECT_FALSE(time_lord.ready_to_terminate());

    executor->run_step();

    EXPECT_TRUE(time == time::Timestamp(std::chrono::seconds(ii)));
  }

  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(LocationConditionUnitTest, TestSameDelay) {
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};
  time::Timestamp time;

  setup_location_condition_test(
      InOut{executor_builder},
      InOut{time_lord},
      InOut{time},
      START_POSES,
      END_POSES,
      END_POSES,
      ACTOR_IDS,
      std::vector<time::Duration>{HALF_SECOND, HALF_SECOND});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // TEST + VERIFY

  // Should run first two steps.
  for (int ii = 0; ii < 2; ++ii) {
    EXPECT_FALSE(time_lord.ready_to_terminate());

    executor->run_step();

    EXPECT_TRUE(time == time::Timestamp(std::chrono::seconds(ii)));
  }

  // Should run exactly one step, 500 milliseconds after update at t=1s.
  executor->run_step();
  EXPECT_TRUE(time == time::Timestamp(ONE_SECOND + HALF_SECOND));

  // Should terminate without reaching t=2s update.
  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(LocationConditionUnitTest, TestDifferentDelay) {
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};
  time::Timestamp time;

  setup_location_condition_test(
      InOut{executor_builder},
      InOut{time_lord},
      InOut{time},
      START_POSES,
      END_POSES,
      END_POSES,
      ACTOR_IDS,
      std::vector<time::Duration>{HALF_SECOND, THREE_QUARTER_SECOND});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // TEST + VERIFY
  // Should run first two steps.
  for (int ii = 0; ii < 2; ++ii) {
    EXPECT_FALSE(time_lord.ready_to_terminate());

    executor->run_step();

    EXPECT_TRUE(time == time::Timestamp(std::chrono::seconds(ii)));
  }

  executor->run_step();

  // Should run exactly one step, 500 milliseconds after update at t=1s.
  EXPECT_TRUE(time == time::Timestamp(ONE_SECOND + HALF_SECOND));

  // Should terminate without reaching t=1.75s or t=2.0s update.
  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(LocationConditionUnitTest, TestDelayBeyondTimeLimit) {
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};
  time::Timestamp time;

  setup_location_condition_test(
      InOut{executor_builder},
      InOut{time_lord},
      InOut{time},
      START_POSES,
      END_POSES,
      END_POSES,
      ACTOR_IDS,
      std::vector<time::Duration>{TWO_SECONDS, TWO_SECONDS});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // Should run only three original steps, as delay would go beyond time limit.
  for (int ii = 0; ii < 3; ++ii) {
    EXPECT_FALSE(time_lord.ready_to_terminate());

    executor->run_step();

    EXPECT_TRUE(time == time::Timestamp(std::chrono::seconds(ii)));
  }
}

TEST(LocationConditionUnitTest, TestSingleUnit) {
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};
  time::Timestamp time;

  setup_location_condition_test(
      InOut{executor_builder},
      InOut{time_lord},
      InOut{time},
      START_POSES,
      MIXED_POSES,
      END_POSES,
      ACTOR_IDS,
      std::vector<time::Duration>{NO_DURATION, NO_DURATION});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // TEST + VERIFY

  // Should run original two steps
  for (int ii = 0; ii < 2; ++ii) {
    EXPECT_FALSE(time_lord.ready_to_terminate());

    executor->run_step();

    EXPECT_TRUE(time == time::Timestamp(std::chrono::seconds(ii)));
  }

  // Should still end after two steps, due to OR semantics, even though one
  // actor has not reached goal.
  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(LocationConditionUnitTest, TestTolerance) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};
  time::Timestamp time{};

  setup_location_condition_test(
      InOut{executor_builder},
      InOut{time_lord},
      InOut{time},
      END_POSES_OFFSET_BY_ONE_METER,
      END_POSES_OFFSET_BY_POINT_ONE_METERS,
      END_POSES,
      ACTOR_IDS,
      std::vector<time::Duration>{NO_DURATION, NO_DURATION},
      HALF_M);

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // TEST + VERIFY

  // Should run exactly two of the updates, skipping the third.
  // First is outside goal tolerance, second is not exact but inside tolerance.
  for (int ii = 0; ii < 2; ++ii) {
    EXPECT_FALSE(time_lord.ready_to_terminate());

    executor->run_step();

    EXPECT_TRUE(time == time::Timestamp(std::chrono::seconds(ii)));
  }

  EXPECT_TRUE(time_lord.ready_to_terminate());
}
TEST(LocationConditionUnitTest, TestWrongIds) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};
  time::Timestamp time{};

  setup_location_condition_test(
      InOut{executor_builder},
      InOut{time_lord},
      InOut{time},
      END_POSES,
      END_POSES,
      END_POSES,
      std::vector<ActorId>{ActorId::new_uuid(), ActorId::new_uuid()},
      std::vector<time::Duration>{NO_DURATION, NO_DURATION});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // TEST + VERIFY

  // Should run original three steps, as no actor IDs match
  for (int ii = 0; ii < 3; ++ii) {
    EXPECT_FALSE(time_lord.ready_to_terminate());

    executor->run_step();

    EXPECT_TRUE(time == time::Timestamp(std::chrono::seconds(ii)));
  }

  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(LocationConditionUnitTest, TestImmediateEnd) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};
  time::Timestamp time{};

  setup_location_condition_test(
      InOut{executor_builder},
      InOut{time_lord},
      InOut{time},
      START_POSES,
      END_POSES,
      START_POSES,
      ACTOR_IDS,
      std::vector<time::Duration>{NO_DURATION, NO_DURATION});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};

  // TEST + VERIFY

  // Should run exactly one update, terminating immediately.
  EXPECT_FALSE(time_lord.ready_to_terminate());

  executor->run_step();

  EXPECT_TRUE(time == time::Timestamp(NO_DURATION));

  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(LocationConditionUnitTest, TestSameDelayAsDt) {
  // SETUP
  ExecutorBuilder executor_builder;
  TimeLord time_lord{nullptr, InOut{executor_builder}};
  time::Timestamp time{};

  setup_location_condition_test(
      InOut{executor_builder},
      InOut{time_lord},
      InOut{time},
      START_POSES,
      END_POSES,
      END_POSES,
      ACTOR_IDS,
      std::vector<time::Duration>{ONE_SECOND, ONE_SECOND});

  std::unique_ptr<StepExecutor> executor{executor_builder.build()};
  // TEST + VERIFY

  // Should run all three updates, but no more.
  for (int ii = 0; ii < 3; ++ii) {
    EXPECT_FALSE(time_lord.ready_to_terminate());

    executor->run_step();

    EXPECT_TRUE(time == time::Timestamp(std::chrono::seconds(ii)));
  }

  EXPECT_TRUE(time_lord.ready_to_terminate());
}

TEST(LocationConditionUnitTest, TestNegativeDelay) {
  // SETUP
  ExecutorBuilder executor_builder;
  std::vector<Condition> conditions{Condition{
      .condition =
          LocationCondition{
              .triggering_actor = ActorId::new_uuid(),
              .target_position = transforms::SE3::identity(),
              .tolerance_m = 0.0,
          },
      .delay = time::Duration(std::chrono::nanoseconds(-1))}};

  const CompletionCriteria completion_criteria{TIME_LIMIT, conditions};

  // TEST + VERIFY
  EXPECT_THROW(
      LocationConditionUnit(
          nullptr,
          completion_criteria,
          InOut{executor_builder}),
      AssertException);
}

TEST(LocationConditionUnitTest, TestNegativeTolerance) {
  // SETUP
  ExecutorBuilder executor_builder;
  std::vector<Condition> conditions{Condition{
      .condition =
          LocationCondition{
              .triggering_actor = ActorId::new_uuid(),
              .target_position = transforms::SE3::identity(),
              .tolerance_m = -1.0,
          },
      .delay = time::Duration(ONE_SECOND)}};

  const CompletionCriteria completion_criteria{TIME_LIMIT, conditions};

  // TEST + VERIFY
  EXPECT_THROW(
      LocationConditionUnit(
          nullptr,
          completion_criteria,
          InOut{executor_builder}),
      AssertException);
}

}  // namespace resim::actor
