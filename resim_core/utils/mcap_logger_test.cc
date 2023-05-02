
#include "resim_core/utils/mcap_logger.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <fstream>
#include <mcap/reader.hpp>
#include <memory>
#include <sstream>

#include "resim_core/assert/assert.hh"
#include "resim_core/testing/test_directory.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/utils/proto/testing/message_a.pb.h"
#include "resim_core/utils/proto/testing/test.pb.h"

namespace resim {

using proto::testing::MessageA;
using TestMsg = proto::testing::Test;

using std::literals::chrono_literals::operator""s;

namespace {

// Checks that the given filepath's contents exactly match the given string.
void expect_file_contains_string(
    const std::filesystem::path &filepath,
    const std::string &string) {
  std::ostringstream expected_os;
  std::ifstream is{filepath};
  expected_os << is.rdbuf();
  EXPECT_EQ(expected_os.str(), string);
}

}  // namespace

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(McapLoggerTest, TestAddProtoChannel) {
  // SETUP
  const testing::TestDirectoryRAII test_directory;
  const std::filesystem::path test_mcap{test_directory.test_file_path("mcap")};

  // ACTION
  constexpr auto TOPIC_A = "/topic_a";
  constexpr auto TOPIC_B = "/topic_b";
  {
    const std::unique_ptr<LoggerInterface> logger{
        std::make_unique<McapLogger>(test_mcap)};
    logger->add_proto_channel<TestMsg>(TOPIC_A);
    logger->add_proto_channel<TestMsg>(TOPIC_B);
  }
  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(test_mcap.string()).ok());

  // VERIFICATION
  // Verify the header
  ASSERT_TRUE(reader.header().has_value());
  constexpr auto EXPECTED_PROFILE = "resim_mcap";
  constexpr auto EXPECTED_ENCODING = "protobuf";
  EXPECT_EQ(EXPECTED_PROFILE, reader.header()->profile);

  // Verify the channels were written
  ASSERT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  const auto &channels = reader.channels();
  ASSERT_EQ(channels.size(), 2U);

  bool found_topic_a = false;
  bool found_topic_b = false;
  for (const auto &[key, val] : channels) {
    if (val->topic == TOPIC_A) {
      found_topic_a = true;
    } else if (val->topic == TOPIC_B) {
      found_topic_b = true;
    }
    EXPECT_EQ(val->messageEncoding, EXPECTED_ENCODING);
  }
  EXPECT_TRUE(found_topic_a and found_topic_b);

  // Verify the schema was written
  const auto &schemas = reader.schemas();
  ASSERT_EQ(schemas.size(), 1U);
  EXPECT_EQ(
      schemas.cbegin()->second->name,
      TestMsg{}.GetDescriptor()->full_name());
  EXPECT_EQ(schemas.cbegin()->second->encoding, EXPECTED_ENCODING);
  const std::string expected_data_string =
      dependency_file_descriptor_set(*TestMsg::GetDescriptor());
  std::vector<std::byte> expected_data{expected_data_string.size()};
  std::memcpy(
      expected_data.data(),
      expected_data_string.data(),
      expected_data_string.size());
  EXPECT_EQ(schemas.cbegin()->second->data, expected_data);

  // Clean up the mcap
  reader.close();
}
// NOLINTEND(readability-function-cognitive-complexity)

TEST(McapLoggerTest, TestLogProto) {
  // SETUP
  const testing::TestDirectoryRAII test_directory;
  const std::filesystem::path test_mcap{test_directory.test_file_path("mcap")};

  // String stream for testing the streaming constructor
  std::ostringstream os;

  constexpr auto TOPIC_A = "/topic_a";
  constexpr auto TOPIC_B = "/topic_b";
  constexpr auto TOPIC_C = "/topic_c";
  constexpr time::Timestamp LOG_TIME{3s};
  constexpr int MESSAGES_PER_CHANNEL = 47;

  // ACTION
  const auto &log_test_messages = [&](auto &&...logger_args) {
    const std::unique_ptr<LoggerInterface> logger{std::make_unique<McapLogger>(
        std::forward<decltype(logger_args)>(logger_args)...)};
    logger->add_proto_channel<TestMsg>(TOPIC_A);
    logger->add_proto_channel<MessageA>(TOPIC_B);
    logger->add_proto_channel<MessageA>(TOPIC_C);
    for (int ii = 0; ii < MESSAGES_PER_CHANNEL; ++ii) {
      logger->log_proto(TOPIC_A, LOG_TIME, TestMsg{});
      logger->log_proto(TOPIC_B, LOG_TIME, MessageA{});
      logger->log_proto(TOPIC_C, LOG_TIME, MessageA{});
    }
  };

  log_test_messages(test_mcap);
  log_test_messages(os);

  // VERIFICATION
  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(test_mcap.string()).ok());

  // VERIFICATION
  // Check that we have exactly one message published
  int count = 0;
  for (const mcap::MessageView &view : reader.readMessages()) {
    EXPECT_EQ(view.message.logTime, LOG_TIME.time_since_epoch().count());
    EXPECT_EQ(view.message.publishTime, LOG_TIME.time_since_epoch().count());
    ++count;
  }
  constexpr int NUM_CHANNELS = 3;
  EXPECT_EQ(count, NUM_CHANNELS * MESSAGES_PER_CHANNEL);

  // Check that the stream contains the same stuff as the file
  expect_file_contains_string(test_mcap, os.str());

  // Clean up the mcap
  reader.close();
}

TEST(McapLoggerDeathTest, TestDoubleAddChannel) {
  constexpr auto CHANNEL = "channel";
  constexpr auto TOPIC_A = "topicA";
  const testing::TestDirectoryRAII test_directory;
  const std::filesystem::path test_mcap{test_directory.test_file_path("mcap")};

  const std::unique_ptr<LoggerInterface> logger{
      std::make_unique<McapLogger>(test_mcap)};

  logger->add_proto_channel<proto::testing::Test>(CHANNEL);
  logger->add_proto_channel<proto::testing::Test>(CHANNEL);  // Double add.

  // Try adding "channel" but haven't seen type MessageA.
  EXPECT_THROW(
      { logger->add_proto_channel<proto::testing::MessageA>(CHANNEL); },
      AssertException);

  // Adding a channel of type MessageA.
  logger->add_proto_channel<proto::testing::MessageA>(TOPIC_A);
  logger->add_proto_channel<proto::testing::MessageA>(TOPIC_A);  // Double add.

  // Try adding "channel" of type MessageA.
  EXPECT_THROW(
      { logger->add_proto_channel<proto::testing::MessageA>(CHANNEL); },
      AssertException);
}

TEST(McapLoggerDeathTest, TestBadFilePath) {
  const testing::TestDirectoryRAII test_directory;
  const std::filesystem::path bad_test_mcap{
      test_directory.path() / "some_bogus_folder" / "test.mcap"};
  EXPECT_THROW({ McapLogger logger{bad_test_mcap}; }, AssertException);
}

TEST(McapLoggerDeathTest, TestBadLogProto) {
  // SETUP
  const testing::TestDirectoryRAII test_directory;
  const std::filesystem::path test_mcap{test_directory.test_file_path("mcap")};
  constexpr auto TOPIC = "/topic";
  constexpr auto BAD_TOPIC = "/bad_topic";
  constexpr time::Timestamp LOG_TIME{3s};

  // ACTION
  const std::unique_ptr<LoggerInterface> logger{
      std::make_unique<McapLogger>(test_mcap)};
  logger->add_proto_channel<TestMsg>(TOPIC);

  EXPECT_THROW(
      logger->log_proto(BAD_TOPIC, LOG_TIME, TestMsg{}),
      AssertException);
  EXPECT_THROW(logger->log_proto(TOPIC, LOG_TIME, MessageA{}), AssertException);
  constexpr auto TOPIC_A = "/topic_a";
  logger->add_proto_channel<MessageA>(TOPIC_A);
  EXPECT_THROW(logger->log_proto(TOPIC, LOG_TIME, MessageA{}), AssertException);
}

}  // namespace resim
