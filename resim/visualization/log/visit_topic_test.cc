// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/log/visit_topic.hh"

#include <gtest/gtest.h>

#include <cstring>
#include <mcap/reader.hpp>
#include <sstream>

#include "resim/assert/assert.hh"
#include "resim/experiences/proto/experience_to_proto.hh"
#include "resim/visualization/log/test_helpers.hh"

namespace resim::visualization::log {

TEST(VisitTopicTest, TestVisitTopic) {
  // SETUP
  experiences::proto::Experience experience_msg;
  pack(test_experience(), &experience_msg);

  std::istringstream input_log{make_input_log(experience_msg)};

  mcap::McapReader reader;
  StreamReader readable{input_log};
  EXPECT_TRUE(reader.open(readable).ok());

  // ACTION
  int exp_count = 0;
  visit_topic("/experience", InOut{reader}, [&](const mcap::MessageView& view) {
    ++exp_count;

    // VERIFICATION
    EXPECT_EQ(
        view.schema->name,
        experiences::proto::Experience::GetDescriptor()->full_name());
    EXPECT_EQ(view.channel->topic, "/experience");

    const std::string expected{experience_msg.SerializeAsString()};
    ASSERT_EQ(view.message.dataSize, expected.size());
    EXPECT_EQ(
        std::memcmp(view.message.data, expected.data(), expected.size()),
        0);
  });
  EXPECT_EQ(exp_count, 1);
}

TEST(VisitTopicTest, TestFailOnNoOpen) {
  // SETUP
  mcap::McapReader reader;

  // ACTION / VERIFICATION
  EXPECT_THROW(
      visit_topic(
          "/some_topic",
          InOut{reader},
          [&](const mcap::MessageView& view) {}),
      AssertException);
}

}  // namespace resim::visualization::log
