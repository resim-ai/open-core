#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <mcap/mcap.hpp>

#include "resim_core/testing/test_directory.hh"
#include "resim_core/utils/uuid.hh"

namespace resim {

mcap::Timestamp arbitrary_time() {
  constexpr unsigned NANOSECONDS = 60234U;
  return mcap::Timestamp{NANOSECONDS};
}

TEST(McapTest, TestWriteAndRead) {
  // SETUP
  const testing::TestDirectoryRAII test_directory;
  const std::filesystem::path test_mcap{test_directory.test_file_path("mcap")};

  constexpr auto TEST_PROFILE = "test_profile";

  mcap::McapWriter writer;
  auto status =
      writer.open(test_mcap.string(), mcap::McapWriterOptions(TEST_PROFILE));

  mcap::Schema test_schema{"uuid", "test_encoding", "test_data"};
  writer.addSchema(test_schema);  // Sets the schema id

  mcap::Channel test_channel("/test_topic", "test_encoding", test_schema.id);
  writer.addChannel(test_channel);  // Sets the channel id

  UUID test_uuid{UUID::new_uuid()};

  // Clang-tidy doesn't like using reinterpret_cast, so we copy this.
  std::array<std::byte, UUID::ARRAY_SIZE> data_buffer{};
  std::memcpy(data_buffer.data(), test_uuid.id().data(), UUID::ARRAY_SIZE);

  mcap::Message test_message;
  test_message.channelId = test_channel.id;
  test_message.sequence = 1;
  test_message.logTime = arbitrary_time();
  test_message.publishTime = test_message.logTime;
  test_message.data = data_buffer.data(),
  test_message.dataSize = test_uuid.id().size();

  // ACTION: Write an MCAP
  ASSERT_TRUE(status.ok());
  ASSERT_TRUE(writer.write(test_message).ok());
  writer.close();

  // ACTION: Read an MCAP
  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(test_mcap.string()).ok());

  // VERIFICATION

  // Verify the header
  ASSERT_TRUE(reader.header().has_value());
  EXPECT_EQ(TEST_PROFILE, reader.header()->profile);

  // Verify the channels
  ASSERT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  const auto &channels = reader.channels();
  ASSERT_EQ(channels.size(), 1U);
  EXPECT_EQ(channels.cbegin()->first, test_channel.id);
  EXPECT_EQ(channels.cbegin()->second->topic, test_channel.topic);
  EXPECT_EQ(
      channels.cbegin()->second->messageEncoding,
      test_channel.messageEncoding);
  EXPECT_EQ(channels.cbegin()->second->schemaId, test_channel.schemaId);
  EXPECT_EQ(channels.cbegin()->second->metadata, test_channel.metadata);

  // Verify the schemas
  const auto &schemas = reader.schemas();
  ASSERT_EQ(schemas.size(), 1U);
  EXPECT_EQ(schemas.cbegin()->first, test_schema.id);
  EXPECT_EQ(schemas.cbegin()->second->name, test_schema.name);
  EXPECT_EQ(schemas.cbegin()->second->encoding, test_schema.encoding);
  EXPECT_EQ(schemas.cbegin()->second->data, test_schema.data);

  // Verifiy the messages
  int count = 0;
  for (const mcap::MessageView &view : reader.readMessages()) {
    EXPECT_EQ(view.message.channelId, test_message.channelId);
    EXPECT_EQ(view.message.sequence, test_message.sequence);
    EXPECT_EQ(view.message.logTime, test_message.logTime);
    EXPECT_EQ(view.message.publishTime, test_message.publishTime);
    std::array<unsigned char, UUID::ARRAY_SIZE> uuid_data{};
    std::memcpy(uuid_data.data(), view.message.data, UUID::ARRAY_SIZE);
    EXPECT_EQ(UUID(uuid_data), test_uuid);
    EXPECT_EQ(view.message.dataSize, test_message.dataSize);
    ++count;
  }
  EXPECT_EQ(count, 1);

  // Clean up the mcap
  reader.close();
}

}  // namespace resim
