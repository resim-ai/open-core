#include "resim/experiences/proto/experience_to_proto.hh"

#include <gtest/gtest.h>

#include "resim/assert/assert.hh"
#include "resim/experiences/experience.hh"
#include "resim/experiences/geometry.hh"
#include "resim/experiences/proto/dynamic_behavior_to_proto.hh"
#include "resim/experiences/proto/experience.pb.h"
#include "resim/experiences/proto/experiences_test_helpers.hh"
#include "resim/experiences/proto/geometry_to_proto.hh"

namespace resim::experiences {

// NOLINTBEGIN(readability-function-cognitive-complexity)
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

  ASSERT_EQ(
      test_experience.geometries.size(),
      experience_msg.geometries_size());
  for (const auto& geometry_msg : experience_msg.geometries()) {
    const Geometry geometry{unpack(geometry_msg)};
    test_geometry_equality(
        geometry,
        test_experience.geometries.at(geometry.id));
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

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
