// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/ros2/resim_log_from_ros2_test_helpers.hh"

#include <google/protobuf/message.h>

#include <cstring>
#include <map>
#include <mcap/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <tuple>

#include "resim/assert/assert.hh"
#include "resim/msg/fuzz_helpers.hh"
#include "resim/ros2/odometry_from_ros2.hh"
#include "resim/ros2/transform_from_ros2.hh"
#include "resim/testing/fuzz_helpers.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/proto/dependency_file_descriptor_set.hh"
#include "resim/utils/tuple_utils.hh"
#include "rosbag2_storage/storage_options.hpp"

namespace resim::ros2 {

// A struct template storing the type and name of a given channel.
template <typename T>
struct Channel {
  using type = T;
  const char *name;
};

// The set of channels we would like to test.
constexpr auto TEST_CHANNELS = std::make_tuple(
    Channel<msg::TransformArray>{"/transforms"},
    Channel<msg::TransformArray>{"/transforms_also"},
    Channel<msg::Odometry>{"/odom"});

// Nested maps storing Channel -> (Timestamp -> Message)
using MessageMap = std::map<
    std::string,
    std::map<mcap::Timestamp, std::unique_ptr<google::protobuf::Message>>>;

constexpr int MESSAGES_PER_CHANNEL = 50;

// Helper function to make MESSAGES_PER_CHANNEL test messages on each channel.
MessageMap make_test_messages() {
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  MessageMap result;
  for_each_in_tuple(
      [&rng, &result](auto channel) {
        using MessageType = typename decltype(channel)::type;
        for (int ii = 0; ii < MESSAGES_PER_CHANNEL; ++ii) {
          const mcap::Timestamp stamp =
              resim::random_element<uint32_t>(InOut{rng});
          REASSERT(result[channel.name]
                       .emplace(
                           stamp,
                           std::make_unique<MessageType>(
                               random_element<MessageType>(InOut{rng})))
                       .second);
        }
        // Need to return something even though we don't use it.
        return true;
      },
      TEST_CHANNELS);
  return result;
}

const MessageMap test_messages = make_test_messages();

// Helper function to populate a ROS2 log with arbitrary test messages on our
// specified channels.
void populate_log(const std::filesystem::path &log_path) {
  rosbag2_cpp::Writer writer;

  rosbag2_storage::StorageOptions storage_options{
      .uri = log_path.string(),
      .storage_id = "mcap",
  };
  writer.open(storage_options);

  for_each_in_tuple(
      [&writer](auto channel) {
        using ReSimType = typename decltype(channel)::type;
        using Ros2Type = decltype(convert_to_ros2(std::declval<ReSimType>()));
        for (const auto &[time, msg_ptr] : test_messages.at(channel.name)) {
          const auto &resim_msg = static_cast<const ReSimType &>(*msg_ptr);
          const Ros2Type ros2_msg = convert_to_ros2(resim_msg);

          const time::SecsAndNanos secs_and_nanos{
              time::to_seconds_and_nanos(time::Duration{time})};
          const rclcpp::Time ros2_time{
              static_cast<int32_t>(secs_and_nanos.secs),
              static_cast<uint32_t>(secs_and_nanos.nanos)};
          writer.write(ros2_msg, channel.name, ros2_time);
        }
        // Need to return something even though we don't use it.
        return true;
      },
      TEST_CHANNELS);

  // Write a message type we don't support converting
  std_msgs::msg::Byte byte;
  writer.write(byte, "/should_not_be_converted", rclcpp::Time());

  std_msgs::msg::UInt8MultiArray arr;
  writer.write(arr, "/should_fail_when_converted", rclcpp::Time());
}

// Helper function to confirm that the ReSim log at the given path contains the
// same messages as the test_messages map.
void verify_log_contents(const std::filesystem::path &log_path) {
  REASSERT(std::filesystem::exists(log_path));

  mcap::McapReader reader;
  REASSERT(reader.open(log_path.c_str()).ok());

  for (const auto &view : reader.readMessages()) {
    const std::string &topic = view.channel->topic;
    const std::string &type = view.schema->name;
    const mcap::Timestamp time = view.message.logTime;
    REASSERT(test_messages.contains(topic));
    REASSERT(time == view.message.publishTime);
    REASSERT(test_messages.at(topic).contains(time));

    // Check that the messages match what's expected.
    for_each_in_tuple(
        [&](auto channel) {
          if (channel.name == topic) {
            using ReSimType = typename decltype(channel)::type;
            ReSimType logged_message;
            REASSERT(logged_message.ParseFromArray(
                view.message.data,
                view.message.dataSize));
            auto &expected_message = static_cast<const ReSimType &>(
                *test_messages.at(topic).at(time));

            verify_equality(logged_message, expected_message);
            REASSERT(type == ReSimType::GetDescriptor()->full_name());

            const std::string expected_data{
                dependency_file_descriptor_set(*ReSimType::GetDescriptor())};
            REASSERT(expected_data.size() == view.schema->data.size());
            REASSERT(
                0 == std::memcmp(
                         expected_data.data(),
                         view.schema->data.data(),
                         expected_data.size()));
          }
          // Need to return something even though we don't use it.
          return true;
        },
        TEST_CHANNELS);
    REASSERT(topic != "/should_not_be_converted");
  }
}

}  // namespace resim::ros2
