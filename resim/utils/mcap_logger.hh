// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <cstring>
#include <filesystem>
#include <map>
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>
#include <ostream>
#include <string>
#include <utility>

#include "resim/assert/assert.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/proto/dependency_file_descriptor_set.hh"

namespace resim {

class LoggerInterface {
 public:
  LoggerInterface() = default;
  LoggerInterface(const LoggerInterface &) = delete;
  LoggerInterface &operator=(const LoggerInterface &) = delete;
  LoggerInterface(LoggerInterface &&) = delete;
  LoggerInterface &operator=(const LoggerInterface &&) = delete;
  virtual ~LoggerInterface() = default;

  // Add a new channel to this logger.
  // @param[in] channel_name - A string identifiying the new channel to
  // create.
  template <typename MessageType>
  void add_proto_channel(const std::string &channel_name) {
    add_proto_channel_impl(*MessageType::GetDescriptor(), channel_name);
  }

  // Log a message.
  // @param[in] channel_name - The channel to log to.
  // @param[in] time - The timestamp (i.e. log time) for this message.
  // @param[in] message - The message itself. Must match the type for this
  //                      channel.
  virtual void log_proto(
      const std::string &channel_name,
      time::Timestamp time,
      const google::protobuf::Message &message) = 0;

 protected:
  // The implementation for adding a new channel.
  // @param[in] message_descriptor - The descriptor for the message type to make
  //                                 a channel for.
  // @param[in] channel_name - A string identifiying the new channel to
  //                           create.
  virtual void add_proto_channel_impl(
      const ::google::protobuf::Descriptor &message_descriptor,
      const std::string &channel_name) = 0;
};

//
// This class is responsible for writing protobuf messages to an mcap file. To
// use the logger, users should set up a channel for each stream of data they
// want to log with the add_proto_channel() member function, and then log
// protobuf messages to these channels with the log_proto() member function.
//
class McapLogger final : public LoggerInterface {
 public:
  // Construct this logger, setting up a log at the given path.
  // @param[in] mcap_path - The path of the file to log to
  explicit McapLogger(const std::filesystem::path &mcap_path);

  // Construct this logger to output to the given ostream. The user is
  // responsible for ensuring this stream's lifetime is longer than this
  // logger's.
  // @param[in] os - The stream to output the binary log to.
  explicit McapLogger(std::ostream &os);

  // Add the contents of an existing log to this log. Channel collisions are
  // *not* allowed and will result in an exception.
  // @param[in] reader - The reader to get messages from.
  // @throws AssertException in the event of a channel collision
  void add_log_contents(InOut<mcap::McapReader> reader);

  McapLogger(const McapLogger &) = delete;
  McapLogger &operator=(const McapLogger &) = delete;
  McapLogger(McapLogger &&) = delete;
  McapLogger &operator=(const McapLogger &&) = delete;
  ~McapLogger() override;

  void log_proto(
      const std::string &channel_name,
      time::Timestamp time,
      const google::protobuf::Message &message) override;

 protected:
  void add_proto_channel_impl(
      const ::google::protobuf::Descriptor &message_descriptor,
      const std::string &channel_name) override;

 private:
  // A helper to add a new schema. A channel's schema is analogous to the
  // message type on that channel.
  void add_proto_schema(
      const ::google::protobuf::Descriptor &message_descriptor,
      const std::string &message_name);

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

}  // namespace resim
