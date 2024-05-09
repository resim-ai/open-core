
#include "resim/visualization/log/test_helpers.hh"

#include <gtest/gtest.h>

#include <mcap/reader.hpp>
#include <variant>

#include "resim/experiences/experience.hh"
#include "resim/experiences/proto/experience.pb.h"
#include "resim/geometry/wireframe.hh"
#include "resim/utils/mcap_logger.hh"
#include "resim/utils/uuid.hh"

namespace resim::visualization::log {

TEST(TestHelpersTest, TestExperience) {
  // SETUP / ACTION
  const experiences::Experience experience{test_experience()};

  // VERIFICATION
  ASSERT_EQ(experience.dynamic_behavior.actors.size(), 1U);
  ASSERT_EQ(experience.dynamic_behavior.actors.front().geometries.size(), 1U);

  const UUID geometry_id{experience.dynamic_behavior.actors.front()
                             .geometries.front()
                             .geometry_id};
  EXPECT_TRUE(experience.geometries.contains(geometry_id));
  EXPECT_EQ(experience.geometries.at(geometry_id).id, geometry_id);
  EXPECT_TRUE(std::holds_alternative<geometry::Wireframe>(
      experience.geometries.at(geometry_id).model));
}

TEST(TestHelpersTest, TestStreamReader) {
  // SETUP
  std::ostringstream sstream;
  std::istringstream input_log{
      make_input_log(experiences::proto::Experience())};

  // ACTION
  StreamReader stream_reader{input_log};
  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(stream_reader).ok());
  ASSERT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  ASSERT_EQ(reader.channels().size(), 1U);
  EXPECT_EQ(reader.channels().cbegin()->second->topic, "/experience");
}

// This is mainly here for coverage purproses. Primary test is that the reader
// works with the mcap::McapReader above.
TEST(TestHelpersTest, TestOffsetGreaterThanSize) {
  std::stringstream sstream{""};
  StreamReader stream_reader{sstream};
  EXPECT_EQ(stream_reader.read(nullptr, 1, 0), 0);
}

}  // namespace resim::visualization::log
