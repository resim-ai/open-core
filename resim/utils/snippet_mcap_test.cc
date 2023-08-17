// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/utils/snippet_mcap.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>
#include <sstream>

#include "resim/utils/inout.hh"
#include "resim/utils/mcap_logger.hh"
#include "resim/utils/mcap_test_helpers.hh"
#include "resim/utils/proto/testing/message_a.pb.h"
#include "resim/utils/proto/testing/message_b.pb.h"
#include "resim/utils/proto/testing/message_c.pb.h"

namespace resim {
using std::literals::chrono_literals::operator""s;
using std::literals::chrono_literals::operator""ns;

constexpr time::Timestamp START_TIME;
constexpr time::Timestamp END_TIME{10s};

// Create a test log containing:
// One message on channel_a with type MessageA before the interval
// One message on chanenl_b_out with type MessageB before the interval
// One message on chanenl_b_in with type MessageB before the interval
// Two messages on chanenl_b_in with type MessageB in the interval
// One message on chanenl_c with type MessageC in the interval
// One message on chanenl_c with type MessageC after the interval
std::string make_test_log() {
  std::ostringstream sstream;

  // Separate scope so the logger gets flushed
  {
    McapLogger logger{sstream};

    logger.add_proto_channel<proto::testing::MessageA>("channel_a");
    logger.log_proto("channel_a", START_TIME - 1s, proto::testing::MessageA());

    logger.add_proto_channel<proto::testing::MessageB>("channel_b_out");
    logger.log_proto(
        "channel_b_out",
        START_TIME - 1ns,
        proto::testing::MessageB());

    logger.add_proto_channel<proto::testing::MessageB>("channel_b_in");
    logger.log_proto(
        "channel_b_in",
        START_TIME - 1ns,
        proto::testing::MessageB());
    logger.log_proto("channel_b_in", START_TIME, proto::testing::MessageB());
    logger.log_proto(
        "channel_b_in",
        START_TIME + 1ns,
        proto::testing::MessageB());

    logger.add_proto_channel<proto::testing::MessageC>("channel_c");
    logger.log_proto("channel_c", END_TIME - 1ns, proto::testing::MessageC());
    logger.log_proto("channel_c", END_TIME, proto::testing::MessageC());
  }
  return sstream.str();
}

// Verify that the snippeted log contains:
// Two messages on chanenl_b_in with type MessageB in the interval
// One message on chanenl_c with type MessageC in the interval
void verify_snippeted_log(const std::string &log) {
  std::istringstream istream{log};
  mcap::McapReader reader;
  StreamReader streamreader{istream};
  ASSERT_TRUE(reader.open(streamreader).ok());

  int message_count = 0;
  for (const auto &view : reader.readMessages()) {
    (void)view;
    ++message_count;
  }
  EXPECT_EQ(message_count, 3);  // Two from channel_b_in and one from channel_c

  std::set<std::string> schemas;
  std::set<std::string> channels;

  for (const auto &[id, schema_ptr] : reader.schemas()) {
    schemas.insert(schema_ptr->name);
  }

  for (const auto &[id, channel_ptr] : reader.channels()) {
    channels.insert(channel_ptr->topic);
  }
  EXPECT_EQ(schemas.size(), 2U);  // MessageB and MessageC

  EXPECT_EQ(channels.size(), 2U);  // channel_b_in and channel_c
}

TEST(SnippetMcapTest, TestSnippeting) {
  // SETUP
  // Input
  const std::string input_log = make_test_log();
  std::istringstream istream{input_log};
  mcap::McapReader input_mcap;
  StreamReader reader{istream};
  ASSERT_TRUE(input_mcap.open(reader).ok());

  // Output
  constexpr auto PROFILE = "resim_mcap";
  const mcap::McapWriterOptions writer_options{PROFILE};
  std::ostringstream ostream;
  mcap::McapWriter output_mcap;
  output_mcap.open(ostream, writer_options);

  // ACTION
  snippet_mcap(START_TIME, END_TIME, InOut{input_mcap}, InOut{output_mcap});
  output_mcap.close();

  // VERIFICATION
  verify_snippeted_log(ostream.str());
}

}  // namespace resim
