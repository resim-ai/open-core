// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/utils/testing/mock_logger.hh"

#include <google/protobuf/message.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "resim/assert/assert.hh"
#include "resim/time/timestamp.hh"

namespace resim::testing {

MockLogger::MockLogger(MockLogger::ChannelToMessageMap &channel_to_message_map)
    : channel_to_message_map_{channel_to_message_map} {}

void MockLogger::log_proto(
    const std::string &channel_name,
    resim::time::Timestamp time,
    const google::protobuf::Message &message) {
  REASSERT(channel_to_name_map_.contains(channel_name));
  REASSERT(
      channel_to_name_map_.at(channel_name) ==
      message.GetDescriptor()->full_name());
  channel_to_message_map_[channel_name].push_back(TimedMessage{
      .time = time,
      .message = message.SerializeAsString(),
  });
}

void MockLogger::add_proto_channel_impl(
    const ::google::protobuf::Descriptor &message_descriptor,
    const std::string &channel_name) {
  if (channel_to_name_map_.contains(channel_name)) {
    REASSERT(
        channel_to_name_map_.at(channel_name) ==
        message_descriptor.full_name());
  }
  channel_to_name_map_[channel_name] = message_descriptor.full_name();
}

}  // namespace resim::testing
