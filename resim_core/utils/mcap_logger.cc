
#include "resim_core/utils/mcap_logger.hh"

namespace resim {

McapLogger::McapLogger(const std::filesystem::path &mcap_path) {
  constexpr auto PROFILE = "resim_mcap";
  const mcap::McapWriterOptions options{PROFILE};
  const auto status = writer_.open(mcap_path.string(), options);
  constexpr auto ERROR_MSG = "Could not open mcap for writing!";
  CHECK(status.ok()) << ERROR_MSG;
}

McapLogger::~McapLogger() { writer_.close(); }

void McapLogger::log_proto(
    const std::string &channel_name,
    const time::Timestamp time,
    const google::protobuf::Message &message) {
  const std::string schema_name{message.GetTypeName()};
  {
    constexpr auto ERR_MSG = "No schema found for this message type!";
    CHECK(schemas_.contains(schema_name)) << ERR_MSG;
  }
  {
    constexpr auto ERR_MSG = "No channel with the given name found!";
    CHECK(channels_.contains(channel_name)) << ERR_MSG;
  }
  {
    const mcap::SchemaId expected_schema_id =
        channel_to_schema_map_.at(channels_.at(channel_name));
    constexpr auto ERR_MSG = "Wrong message type for channel!";
    CHECK(expected_schema_id == schemas_.at(schema_name)) << ERR_MSG;
  }
  mcap::Message msg;
  msg.channelId = channels_.at(channel_name);

  msg.publishTime = time.time_since_epoch().count();
  msg.logTime = time.time_since_epoch().count();

  std::string data_string{message.SerializeAsString()};
  std::vector<std::byte> data_vec{data_string.size()};
  std::memcpy(data_vec.data(), data_string.data(), data_string.size());
  msg.data = data_vec.data();
  msg.dataSize = data_string.size();
  const auto success = writer_.write(msg);
  CHECK(success.ok()) << "Failed to write message!";
}

}  // namespace resim