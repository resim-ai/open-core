// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/utils/mcap_test_helpers.hh"

#include <gtest/gtest.h>

#include <mcap/reader.hpp>

#include "resim/utils/mcap_logger.hh"
#include "resim/utils/proto/testing/message_a.pb.h"

namespace resim::utils {

// Make a test log containing a message
std::string make_test_log() {
  std::ostringstream sstream;

  // Separate scope so the logger gets flushed
  {
    McapLogger logger{sstream};

    logger.add_proto_channel<proto::testing::MessageA>("channel_a");
    logger.log_proto(
        "channel_a",
        time::Timestamp(),
        proto::testing::MessageA());
  }
  return sstream.str();
}

TEST(TestHelpersTest, TestStreamReader) {
  // SETUP
  std::ostringstream sstream;
  std::istringstream input_log{make_test_log()};

  // ACTION
  StreamReader stream_reader{input_log};
  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(stream_reader).ok());
  ASSERT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  ASSERT_EQ(reader.channels().size(), 1U);
  EXPECT_EQ(reader.channels().cbegin()->second->topic, "channel_a");
}

// This is mainly here for coverage purproses. Primary test is that the reader
// works with the mcap::McapReader above.
TEST(TestHelpersTest, TestOffsetGreaterThanSize) {
  std::stringstream sstream{""};
  StreamReader stream_reader{sstream};
  EXPECT_EQ(stream_reader.read(nullptr, 1, 0), 0);
}

}  // namespace resim::utils
