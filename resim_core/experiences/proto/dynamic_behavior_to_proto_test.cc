#include "resim_core/experiences/proto/dynamic_behavior_to_proto.hh"

#include <gtest/gtest.h>

#include "resim_core/assert/assert.hh"
#include "resim_core/experiences/dynamic_behavior.hh"
#include "resim_core/experiences/proto/actor_to_proto.hh"
#include "resim_core/experiences/proto/completion_criteria_to_proto.hh"
#include "resim_core/experiences/proto/dynamic_behavior.pb.h"
#include "resim_core/experiences/proto/experiences_test_helpers.hh"
#include "resim_core/experiences/proto/storyboard_to_proto.hh"

namespace resim::experiences {

TEST(DynamicBehaviorToProtoTest, TestPack) {
  // SETUP
  DynamicBehavior test_dynamic_behavior = make_test_dynamic_behavior();
  proto::DynamicBehavior dynamic_behavior_msg;

  // ACTION
  pack(test_dynamic_behavior, &dynamic_behavior_msg);
  // VERIFICATION
  ASSERT_EQ(
      test_dynamic_behavior.actors.size(),
      dynamic_behavior_msg.actors().size());
  for (int i = 0; i < dynamic_behavior_msg.actors().size(); i++) {
    Actor actor = test_dynamic_behavior.actors.at(i);
    proto::Actor proto_actor = dynamic_behavior_msg.actors().at(i);
    // Check the actor refeence
    ASSERT_TRUE(test_actor_equality(actor, proto::unpack(proto_actor)));
  }
  ASSERT_TRUE(test_storyboard_equality(
      test_dynamic_behavior.storyboard,
      unpack(dynamic_behavior_msg.storyboard())));
  ASSERT_TRUE(test_completion_criteria_equality(
      test_dynamic_behavior.completion_criteria,
      unpack(dynamic_behavior_msg.completion_criteria())));
}

TEST(DynamicBehaviorToProtoTest, TestRoundTrip) {
  // SETUP
  DynamicBehavior test_dynamic_behavior = make_test_dynamic_behavior();
  proto::DynamicBehavior dynamic_behavior_msg;

  // ACTION
  pack(test_dynamic_behavior, &dynamic_behavior_msg);
  const DynamicBehavior& retrieved_dynamic_behavior =
      unpack(dynamic_behavior_msg);
  test_dynamic_behavior_equality(
      test_dynamic_behavior,
      retrieved_dynamic_behavior);
}

TEST(DynamicBehaviorToProtoDeathTest, TestPackInvalid) {
  // ACTION / VERIFICATION
  EXPECT_THROW(
      proto::pack(make_test_dynamic_behavior(), nullptr),
      AssertException);
}

}  // namespace resim::experiences
