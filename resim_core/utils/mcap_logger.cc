
#include "resim_core/utils/mcap_logger.hh"

namespace resim {

namespace {
constexpr auto PROFILE = "resim_mcap";
}

McapLogger::McapLogger(const std::filesystem::path &mcap_path) {
  const mcap::McapWriterOptions options{PROFILE};
  const auto status = writer_.open(mcap_path.string(), options);
  constexpr auto ERROR_MSG = "Could not open mcap for writing!";
  REASSERT(status.ok(), ERROR_MSG);
}

McapLogger::McapLogger(std::ostream &os) {
  const mcap::McapWriterOptions options{PROFILE};
  writer_.open(os, options);
}

McapLogger::~McapLogger() { writer_.close(); }

void McapLogger::log_proto(
    const std::string &channel_name,
    const time::Timestamp time,
    const google::protobuf::Message &message) {
  const std::string schema_name{message.GetTypeName()};
  {
    constexpr auto ERR_MSG = "No schema found for this message type!";
    REASSERT(schemas_.contains(schema_name), ERR_MSG);
  }
  {
    constexpr auto ERR_MSG = "No channel with the given name found!";
    REASSERT(channels_.contains(channel_name), ERR_MSG);
  }
  {
    const mcap::SchemaId expected_schema_id =
        channel_to_schema_map_.at(channels_.at(channel_name));
    constexpr auto ERR_MSG = "Wrong message type for channel!";
    REASSERT(expected_schema_id == schemas_.at(schema_name), ERR_MSG);
  }
  mcap::Message msg;
  msg.channelId = channels_.at(channel_name);

  msg.publishTime = time.time_since_epoch().count();
  msg.logTime = time.time_since_epoch().count();

  msg.sequence = 0U;

  std::string data_string{message.SerializeAsString()};
  std::vector<std::byte> data_vec{data_string.size()};
  std::memcpy(data_vec.data(), data_string.data(), data_string.size());
  msg.data = data_vec.data();
  msg.dataSize = data_string.size();
  const auto success = writer_.write(msg);
  REASSERT(success.ok(), "Failed to write message!");
}

void McapLogger::add_proto_channel_impl(
    const ::google::protobuf::Descriptor &message_descriptor,
    const std::string &channel_name) {
  const std::string &message_name{message_descriptor.full_name()};

  // If channel_name exists, error if of different MessageType.
  if (channels_.contains(channel_name)) {
    {
      constexpr auto ERR_MSG = "Schema does not exist.";
      REASSERT(schemas_.contains(message_name), ERR_MSG);
    }

    {
      const mcap::SchemaId expected_schema_id =
          channel_to_schema_map_.at(channels_.at(channel_name));
      constexpr auto ERR_MSG =
          "Channel with name but different MessageType already added!";
      REASSERT(expected_schema_id == schemas_.at(message_name), ERR_MSG);
    }

    return;
  }

  add_proto_schema(message_descriptor, message_name);
  constexpr auto ENCODING = "protobuf";
  mcap::Channel channel{channel_name, ENCODING, schemas_.at(message_name)};
  writer_.addChannel(channel);
  channels_.emplace(channel_name, channel.id);
  channel_to_schema_map_.emplace(channel.id, schemas_.at(message_name));
}

void McapLogger::add_proto_schema(
    const ::google::protobuf::Descriptor &message_descriptor,
    const std::string &message_name) {
  if (not schemas_.contains(message_name)) {
    mcap::Schema schema{
        message_name,
        "protobuf",
        dependency_file_descriptor_set(message_descriptor)};
    writer_.addSchema(schema);
    schemas_.emplace(message_name, schema.id);
  }
}

}  // namespace resim
