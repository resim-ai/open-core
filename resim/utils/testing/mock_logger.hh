// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <google/protobuf/message.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "resim/assert/assert.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/mcap_logger.hh"

namespace resim::testing {
// A simple mock of the LoggerInterface which shoves all of the messages into a
// message map. This map is held by reference so that the creator of this logger
// can inspect it.
class MockLogger : public LoggerInterface {
 public:
  struct TimedMessage {
    time::Timestamp time;
    std::string message;
  };

  using ChannelToMessageMap =
      std::unordered_map<std::string, std::vector<TimedMessage>>;

  explicit MockLogger(ChannelToMessageMap &channel_to_message_map);

  void log_proto(
      const std::string &channel_name,
      resim::time::Timestamp time,
      const google::protobuf::Message &message) override;

 protected:
  void add_proto_channel_impl(
      const ::google::protobuf::Descriptor &message_descriptor,
      const std::string &channel_name) override;

 private:
  ChannelToMessageMap &channel_to_message_map_;
  std::unordered_map<std::string, std::string> channel_to_name_map_{};
};
}  // namespace resim::testing
