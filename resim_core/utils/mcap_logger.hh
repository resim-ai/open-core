#pragma once

#include <glog/logging.h>

#include <cstring>
#include <filesystem>
#include <map>
#include <mcap/writer.hpp>
#include <string>
#include <utility>

#include "resim_core/time/timestamp.hh"
#include "resim_core/utils/proto/dependency_file_descriptor_set.hh"

namespace resim {

//
// This class is responsible for writing protobuf messages to an mcap file. To
// use the logger, users should set up a channel for each stream of data they
// want to log with the add_proto_channel() member function, and then log
// protobuf messages to these channels with the log_proto() member function.
//
class McapLogger {
 public:
  // Construct this logger, setting up a log at the given path.
  // @param[in] mcap_path - The path of the file to log to
  explicit McapLogger(const std::filesystem::path &mcap_path);

  McapLogger(const McapLogger &) = delete;
  McapLogger &operator=(const McapLogger &) = delete;
  McapLogger(McapLogger &&) = delete;
  McapLogger &operator=(const McapLogger &&) = delete;
  ~McapLogger();

  // Add a new channel to this logger.
  // @param[in] channel_name - A string identifiying the new channel to create.
  template <typename MessageType>
  void add_proto_channel(const std::string &channel_name);

  // Log a message.
  // @param[in] channel_name - The channel to log to.
  // @param[in] time - The timestamp (i.e. log time) for this message.
  // @param[in] message - The message itself. Must match the type for this
  //                      channel.
  void log_proto(
      const std::string &channel_name,
      time::Timestamp time,
      const google::protobuf::Message &message);

 private:
  // A helper to add a new schema. A channel's schema is analogous to the
  // message type on that channel.
  template <typename MessageType>
  void add_proto_schema(const std::string &message_name);

  // A map of message type names to the id of the schema used by the writer_.
  std::map<std::string, mcap::SchemaId> schemas_;

  // A map of channel names to the id of the channel used by the writer_.
  std::map<std::string, mcap::ChannelId> channels_;

  // A map of channel ids to their corresponding schemas. Used to enforce that
  // logged message types match the channels they are being logged to.
  std::map<mcap::ChannelId, mcap::SchemaId> channel_to_schema_map_;

  // The underlying writer we use to actually produce the mcap
  mcap::McapWriter writer_;
};

template <typename MessageType>
void McapLogger::add_proto_channel(const std::string &channel_name) {
  const std::string message_name{MessageType().GetDescriptor()->full_name()};

  // If channel_name exists, error if of different MessageType.
  if (channels_.contains(channel_name)) {
    {
      constexpr auto ERR_MSG = "Schema does not exist.";
      CHECK(schemas_.contains(message_name)) << ERR_MSG;
    }

    {
      const mcap::SchemaId expected_schema_id =
          channel_to_schema_map_.at(channels_.at(channel_name));
      constexpr auto ERR_MSG =
          "Channel with name but different MessageType already added!";
      CHECK(expected_schema_id == schemas_.at(message_name)) << ERR_MSG;
    }

    return;
  }

  add_proto_schema<MessageType>(message_name);
  constexpr auto ENCODING = "protobuf";
  mcap::Channel channel{channel_name, ENCODING, schemas_.at(message_name)};
  writer_.addChannel(channel);
  channels_.emplace(channel_name, channel.id);
  channel_to_schema_map_.emplace(channel.id, schemas_.at(message_name));
}

template <typename MessageType>
void McapLogger::add_proto_schema(const std::string &message_name) {
  if (not schemas_.contains(message_name)) {
    mcap::Schema schema{
        message_name,
        "protobuf",
        dependency_file_descriptor_set(*MessageType{}.GetDescriptor())};
    writer_.addSchema(schema);
    schemas_.emplace(message_name, schema.id);
  }
}

}  // namespace resim
