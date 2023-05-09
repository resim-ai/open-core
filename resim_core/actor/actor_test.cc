
#include "resim_core/actor/actor.hh"

#include <gtest/gtest.h>

#include <memory>

#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/test_actor.hh"

namespace resim::actor {

TEST(ActorTest, TestConstruction) {
  const ActorId actor_id{ActorId::new_uuid()};
  std::unique_ptr<Actor> actor = std::make_unique<TestActor>(actor_id);
  EXPECT_EQ(actor->id(), actor_id);
}

TEST(ActorTest, TestIdPersistent) {
  const ActorId actor_id{ActorId::new_uuid()};
  std::unique_ptr<Actor> actor = std::make_unique<TestActor>(actor_id);
  const ActorId first_query = actor->id();
  const ActorId second_query = actor->id();
  EXPECT_EQ(first_query, second_query);
}

}  // namespace resim::actor
