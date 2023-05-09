#include "resim_core/experiences/proto/location_condition_to_proto.hh"

#include <gtest/gtest.h>

#include "resim_core/assert/assert.hh"
#include "resim_core/experiences/location_condition.hh"
#include "resim_core/experiences/proto/experiences_test_helpers.hh"
#include "resim_core/experiences/proto/location_condition.pb.h"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/proto/fse3_to_proto.hh"
#include "resim_core/utils/proto/uuid_to_proto.hh"

namespace resim::experiences {

TEST(LocationConditionToProtoTest, TestPack) {
  // SETUP
  const LocationCondition& test_location_condition =
      make_test_location_condition();
  proto::LocationCondition location_condition_msg;

  // ACTION
  pack(test_location_condition, &location_condition_msg);
  // VERIFICATION
  ASSERT_EQ(
      test_location_condition.triggering_actor,
      unpack(location_condition_msg.triggering_actor().id()));
  ASSERT_TRUE(test_location_condition.target_position.is_approx(
      unpack(location_condition_msg.target_position())));
  ASSERT_DOUBLE_EQ(
      test_location_condition.tolerance_m,
      location_condition_msg.tolerance_m());
}

TEST(LocationConditionToProtoTest, TestRoundTrip) {
  // SETUP
  const LocationCondition& test_location_condition =
      make_test_location_condition();
  proto::LocationCondition location_condition_msg;

  // ACTION
  pack(test_location_condition, &location_condition_msg);
  const LocationCondition& retrieved_location_condition =
      proto::unpack(location_condition_msg);
  // VERIFICATION
  test_location_condition_equality(
      test_location_condition,
      retrieved_location_condition);
}

TEST(LocationConditionToProtoDeathTest, TestPackInvalid) {
  // ACTION / VERIFICATION
  EXPECT_THROW(
      proto::pack(make_test_location_condition(), nullptr),
      AssertException);
}

}  // namespace resim::experiences
