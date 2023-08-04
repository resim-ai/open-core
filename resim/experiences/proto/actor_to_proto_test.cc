// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/experiences/proto/actor_to_proto.hh"

#include <gtest/gtest.h>

#include "resim/assert/assert.hh"
#include "resim/experiences/actor.hh"
#include "resim/experiences/proto/actor.pb.h"
#include "resim/experiences/proto/experiences_test_helpers.hh"
#include "resim/utils/proto/uuid_to_proto.hh"

namespace resim::experiences {

// NOLINTBEGIN(readability-function-cognitive-complexity)
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

  ASSERT_EQ(
      simulation_actor.geometries.size(),
      sim_actor_msg.geometries_size());
  for (int ii = 0; ii < sim_actor_msg.geometries_size(); ++ii) {
    EXPECT_EQ(
        simulation_actor.geometries.at(ii).geometry_id.to_string(),
        sim_actor_msg.geometries(ii).geometry_id().data());
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

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
