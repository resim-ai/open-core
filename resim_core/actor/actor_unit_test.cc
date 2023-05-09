
#include "resim_core/actor/actor_unit.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <variant>

#include "resim_core/actor/actor.hh"
#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/geometry.hh"
#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/actor/test_actor.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/simulator/executor_builder.hh"
#include "resim_core/simulator/standard_topics.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/utils/inout.hh"

namespace resim::actor {

// GTEST macros end up causing clang-tidy to overestimate the complexity
// NOLINTBEGIN(readability-function-cognitive-complexity)
void test_actor_unit_valid_state(const transforms::FSE3 &pose) {
  const ActorId id{ActorId::new_uuid()};
  std::unique_ptr<TestActor> actor{std::make_unique<TestActor>(id)};

  bool simulated_forward = false;
  actor->set_simulate_forward([actor = actor.get(), &pose, &simulated_forward](
                                  const time::Timestamp time) {
    const state::ObservableState state{
        .id = actor->id(),
        .is_spawned = true,
        .time_of_validity = time,
        .state = state::RigidBodyState<transforms::FSE3>{pose},
    };
    actor->set_state(state);

    const Geometry geometry{
        .frame = pose.from(),
        .time_of_validity = time,
        .visible_geometry = Geometry::Clear{},
    };
    actor->set_geometry(geometry);
    simulated_forward = true;
  });

  simulator::ExecutorBuilder executor_builder;
  ActorUnit unit{std::move(actor), InOut{executor_builder}};

  constexpr time::Timestamp TIME{std::chrono::seconds(1)};
  executor_builder.add_independent_task<time::Timestamp>(
      "publish_time",
      simulator::TIME_TOPIC,
      [TIME]() { return TIME; });

  bool state_checked = false;
  executor_builder.add_task<state::ObservableState>(
      "check_state",
      simulator::ACTOR_STATES_TOPIC,
      simulator::NULL_TOPIC,
      [&pose, &TIME, &state_checked, &simulated_forward](
          const state::ObservableState &state) {
        EXPECT_TRUE(state.state.ref_from_body().is_approx(pose));
        EXPECT_EQ(state.time_of_validity, TIME);
        state_checked = true;
        EXPECT_TRUE(simulated_forward);
      });

  bool geometry_checked = false;
  executor_builder.add_task<Geometry>(
      "check_geometry",
      simulator::ACTOR_GEOMETRIES_TOPIC,
      simulator::NULL_TOPIC,
      [&pose, &TIME, &geometry_checked, &simulated_forward](
          const Geometry &geometry) {
        EXPECT_TRUE(
            std::holds_alternative<Geometry::Clear>(geometry.visible_geometry));
        EXPECT_EQ(geometry.frame, pose.from());
        EXPECT_EQ(geometry.time_of_validity, TIME);
        geometry_checked = true;
        EXPECT_TRUE(simulated_forward);
      });

  auto executor = executor_builder.build();

  // ACTION
  executor->run_step();

  EXPECT_TRUE(state_checked);
  EXPECT_TRUE(geometry_checked);
}
// NOLINTEND(readability-function-cognitive-complexity)

TEST(ActorUnitTest, TestActorUnitValidState) {
  for (const transforms::FSE3 &pose :
       transforms::make_test_group_elements<transforms::FSE3>()) {
    test_actor_unit_valid_state(pose);
  }
}

void test_actor_unit_bad_state_time(const transforms::FSE3 &pose) {}

TEST(ActorUnitTest, TestActorUnitBadStateTime) {
  const ActorId id{ActorId::new_uuid()};
  std::unique_ptr<TestActor> actor{std::make_unique<TestActor>(id)};

  actor->set_simulate_forward(
      [actor = actor.get()](const time::Timestamp time) {
        const state::ObservableState state{
            .id = actor->id(),
            .is_spawned = true,
            // Put in an incorrect time for this state:
            .time_of_validity = time + std::chrono::nanoseconds(1),
            .state = state::RigidBodyState<transforms::FSE3>{},
        };
        actor->set_state(state);
      });

  simulator::ExecutorBuilder executor_builder;
  ActorUnit unit{std::move(actor), InOut{executor_builder}};

  constexpr time::Timestamp TIME{std::chrono::seconds(1)};
  executor_builder.add_independent_task<time::Timestamp>(
      "publish_time",
      simulator::TIME_TOPIC,
      [TIME]() { return TIME; });

  auto executor = executor_builder.build();

  // ACTION / VERIFICATION
  EXPECT_THROW(executor->run_step(), AssertException);
}

TEST(ActorUnitTest, TestActorUnitBadStateId) {
  const ActorId id{ActorId::new_uuid()};
  std::unique_ptr<TestActor> actor{std::make_unique<TestActor>(id)};

  actor->set_simulate_forward(
      [actor = actor.get()](const time::Timestamp time) {
        const state::ObservableState state{
            .id = ActorId::new_uuid(),  // Wrong ID
            .is_spawned = true,
            .time_of_validity = time,
            .state = state::RigidBodyState<transforms::FSE3>{},
        };
        actor->set_state(state);
      });

  simulator::ExecutorBuilder executor_builder;
  ActorUnit unit{std::move(actor), InOut{executor_builder}};

  constexpr time::Timestamp TIME{std::chrono::seconds(1)};
  executor_builder.add_independent_task<time::Timestamp>(
      "publish_time",
      simulator::TIME_TOPIC,
      [TIME]() { return TIME; });

  auto executor = executor_builder.build();

  // ACTION / VERIFICATION
  EXPECT_THROW(executor->run_step(), AssertException);
}

}  // namespace resim::actor
