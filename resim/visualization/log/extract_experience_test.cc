
#include "resim/visualization/log/extract_experience.hh"

#include <gtest/gtest.h>

#include <mcap/reader.hpp>
#include <sstream>

#include "resim/assert/assert.hh"
#include "resim/experiences/experience.hh"
#include "resim/experiences/proto/experience.pb.h"
#include "resim/experiences/proto/experience_to_proto.hh"
#include "resim/experiences/proto/experiences_test_helpers.hh"
#include "resim/visualization/log/test_helpers.hh"

namespace resim::visualization::log {

TEST(ExtractExperienceTest, TestExtractExperience) {
  // SETUP
  experiences::Experience experience{test_experience()};
  experiences::proto::Experience experience_msg;
  pack(experience, &experience_msg);

  std::istringstream input_log{make_input_log(experience_msg)};

  mcap::McapReader reader;
  StreamReader readable{input_log};
  EXPECT_TRUE(reader.open(readable).ok());

  // ACTION
  experiences::Experience extracted{extract_experience(InOut{reader})};

  // VERIFICATION
  EXPECT_TRUE(experiences::test_experience_equality(extracted, experience));
}

TEST(ExtractExperienceTest, TestExtractExperienceTooFewTooMany) {
  for (const int num_messages : {0, 2}) {
    // SETUP
    std::istringstream input_log{
        make_input_log(experiences::proto::Experience(), num_messages)};

    mcap::McapReader reader;
    StreamReader readable{input_log};
    EXPECT_TRUE(reader.open(readable).ok());

    // ACTION / VERIFICATION
    EXPECT_THROW(extract_experience(InOut{reader}), AssertException);
  }
}

}  // namespace resim::visualization::log
