#include "resim_core/experiences/proto/experience_to_proto.hh"

#include <gtest/gtest.h>

#include "resim_core/assert/assert.hh"
#include "resim_core/experiences/experience.hh"
#include "resim_core/experiences/proto/dynamic_behavior_to_proto.hh"
#include "resim_core/experiences/proto/experience.pb.h"
#include "resim_core/experiences/proto/experiences_test_helpers.hh"

namespace resim::experiences {

TEST(ExperienceToProtoTest, TestPack) {
  // SETUP
  Experience test_experience = make_test_experience();
  proto::Experience experience_msg;

  // ACTION
  pack(test_experience, &experience_msg);
  // VERIFICATION
  ASSERT_TRUE(test_dynamic_behavior_equality(
      test_experience.dynamic_behavior,
      unpack(experience_msg.dynamic_behavior())));

  ASSERT_EQ(
      test_experience.header.revision.major,
      experience_msg.header().experience_schema_revision().major_revision());
  ASSERT_EQ(
      test_experience.header.revision.minor,
      experience_msg.header().experience_schema_revision().minor_revision());
  ASSERT_EQ(test_experience.header.name, experience_msg.header().name());
  ASSERT_EQ(
      test_experience.header.description,
      experience_msg.header().description());
  ASSERT_EQ(
      test_experience.header.parent_experience_name,
      experience_msg.header().parent_experience_name());
}

TEST(ExperienceToProtoTest, TestRoundTrip) {
  // SETUP
  Experience test_experience = make_test_experience();
  proto::Experience experience_msg;

  // ACTION
  pack(test_experience, &experience_msg);
  const Experience& retrieved_experience = unpack(experience_msg);
  test_experience_equality(retrieved_experience, test_experience);
}

TEST(ExperienceToProtoDeathTest, TestPackInvalid) {
  // ACTION / VERIFICATION
  EXPECT_THROW(proto::pack(make_test_experience(), nullptr), AssertException);
}

}  // namespace resim::experiences
