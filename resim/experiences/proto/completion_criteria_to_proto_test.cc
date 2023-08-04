// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/experiences/proto/completion_criteria_to_proto.hh"

#include <gtest/gtest.h>

#include "resim/assert/assert.hh"
#include "resim/experiences/completion_criteria.hh"
#include "resim/experiences/proto/completion_criteria.pb.h"
#include "resim/experiences/proto/experiences_test_helpers.hh"
#include "resim/experiences/proto/location_condition_to_proto.hh"

namespace resim::experiences {

TEST(CompletionCriteriaToProtoTest, TestPack) {
  // SETUP
  CompletionCriteria test_completion_criteria = make_test_completion_criteria();
  proto::CompletionCriteria completion_criteria_msg;

  // ACTION
  pack(test_completion_criteria, &completion_criteria_msg);
  // VERIFICATION
  EXPECT_EQ(
      completion_criteria_msg.time_limit().seconds() * std::nano::den +
          completion_criteria_msg.time_limit().nanos(),
      test_completion_criteria.time_limit.count());
  ASSERT_EQ(
      test_completion_criteria.conditions.size(),
      completion_criteria_msg.conditions().size());
  for (int i = 0; i < completion_criteria_msg.conditions().size(); i++) {
    const proto::Condition& proto_condition =
        completion_criteria_msg.conditions().at(i);
    const Condition& condition = test_completion_criteria.conditions.at(i);
    EXPECT_EQ(
        proto_condition.delay().seconds() * std::nano::den +
            completion_criteria_msg.time_limit().nanos(),
        condition.delay.count());
    EXPECT_TRUE(proto_condition.has_location_condition());
    const proto::LocationCondition& proto_location_condition =
        proto_condition.location_condition();
    // This is broken:
    const LocationCondition& packed_location_condition =
        unpack(proto_location_condition);
    test_location_condition_equality(
        condition.condition,
        packed_location_condition);
  }
}

TEST(CompletionCriteriaToProtoTest, TestRoundTrip) {
  // SETUP
  CompletionCriteria test_completion_criteria = make_test_completion_criteria();
  proto::CompletionCriteria completion_criteria_msg;

  // ACTION
  pack(test_completion_criteria, &completion_criteria_msg);
  const CompletionCriteria& retrieved_completion_criteria =
      unpack(completion_criteria_msg);
  test_completion_criteria_equality(
      test_completion_criteria,
      retrieved_completion_criteria);
}

TEST(CompletionCriteriaToProtoDeathTest, TestPackInvalid) {
  // ACTION / VERIFICATION
  EXPECT_THROW(
      proto::pack(make_test_completion_criteria(), nullptr),
      AssertException);
}

}  // namespace resim::experiences
