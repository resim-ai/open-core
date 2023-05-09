#include "resim_core/experiences/proto/actor_to_proto.hh"

#include <gtest/gtest.h>

#include "resim_core/assert/assert.hh"
#include "resim_core/experiences/actor.hh"
#include "resim_core/experiences/proto/actor.pb.h"
#include "resim_core/experiences/proto/experiences_test_helpers.hh"
#include "resim_core/utils/proto/uuid_to_proto.hh"

namespace resim::experiences {

TEST(ActorToProtoTest, TestPack) {
  // SETUP
  Actor simulation_actor = make_test_actor();
  Actor sut_actor = make_test_actor(ActorType::SYSTEM_UNDER_TEST);
  proto::Actor sim_actor_msg;
  proto::Actor sut_actor_msg;

  // ACTION
  pack(simulation_actor, &sim_actor_msg);
  pack(sut_actor, &sut_actor_msg);

  // VERIFICATION
  // Check the UUIDs:
  EXPECT_EQ(simulation_actor.id, unpack(sim_actor_msg.id()));
  EXPECT_EQ(sut_actor.id, unpack(sut_actor_msg.id()));
  // Check the names
  EXPECT_EQ(simulation_actor.name, sim_actor_msg.name());
  EXPECT_EQ(sut_actor.name, sut_actor_msg.name());
  // Check the types
  ASSERT_EQ(
      sim_actor_msg.actor_type(),
      proto::Actor_ActorType_SIMULATION_ACTOR);
  ASSERT_EQ(
      sut_actor_msg.actor_type(),
      proto::Actor_ActorType_SYSTEM_UNDER_TEST);
}

TEST(ActorToProtoTest, TestRoundTrip) {
  // SETUP
  Actor simulation_actor = make_test_actor();
  Actor sut_actor = make_test_actor(ActorType::SYSTEM_UNDER_TEST);
  proto::Actor sim_actor_msg;
  proto::Actor sut_actor_msg;

  // ACTION
  pack(simulation_actor, &sim_actor_msg);
  pack(sut_actor, &sut_actor_msg);
  const Actor retrieved_sim_actor = proto::unpack(sim_actor_msg);
  const Actor retrieved_sut_actor = proto::unpack(sut_actor_msg);
  // VERIFICATION
  test_actor_equality(simulation_actor, retrieved_sim_actor);
  test_actor_equality(sut_actor, retrieved_sut_actor);
}

TEST(ActorToProtoDeathTest, TestPackInvalid) {
  // ACTION / VERIFICATION
  EXPECT_THROW(proto::pack(make_test_actor(), nullptr), AssertException);
}

TEST(ActorToProtoDeathTest, TestPackInvalidActor) {
  // ACTION / VERIFICATION
  Actor invalid_actor;
  invalid_actor.id = UUID::new_uuid();
  invalid_actor.name = "Invalid Actor";
  proto::Actor invalid_actor_msg;
  invalid_actor.actor_type = ActorType::INVALID;
  EXPECT_THROW(proto::pack(invalid_actor, &invalid_actor_msg), AssertException);
}

TEST(StoryboardToProtoDeathTest, TestUnpackInvalid) {
  // SETUP
  proto::Actor actor;
  pack(UUID::new_uuid(), actor.mutable_id());
  // ACTION / VERIFICATION
  EXPECT_THROW(unpack(actor), AssertException);
}

}  // namespace resim::experiences
