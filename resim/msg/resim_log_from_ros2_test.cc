// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/resim_log_from_ros2.hh"

#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <map>
#include <mcap/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <string>
#include <tuple>

#include "resim/assert/assert.hh"
#include "resim/msg/fuzz_helpers.hh"
#include "resim/msg/odometry_from_ros2.hh"
#include "resim/msg/transform_from_ros2.hh"
#include "resim/testing/fuzz_helpers.hh"
#include "resim/testing/test_directory.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/proto/dependency_file_descriptor_set.hh"
#include "resim/utils/tuple_utils.hh"
#include "rosbag2_storage/storage_options.hpp"

namespace resim::msg {

namespace {

// A struct template storing the type and name of a given channel.
template <typename T>
struct Channel {
  using type = T;
  const char *name;
};

// The set of channels we would like to test.
constexpr auto TEST_CHANNELS = std::make_tuple(
    Channel<TransformArray>{"/transforms"},
    Channel<Odometry>{"/odom"});

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
}

// Helper function to confirm that the ReSim log at the given path contains the
// same messages as the test_messages map.
void verify_log_contents(const std::filesystem::path &log_path) {
  ASSERT_TRUE(std::filesystem::exists(log_path));

  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(log_path.c_str()).ok());

  for (const auto &view : reader.readMessages()) {
    const std::string &topic = view.channel->topic;
    const std::string &type = view.schema->name;
    const mcap::Timestamp time = view.message.logTime;
    ASSERT_TRUE(test_messages.contains(topic));
    EXPECT_EQ(time, view.message.publishTime);
    ASSERT_TRUE(test_messages.at(topic).contains(time));

    // Check that the messages match what's expected.
    for_each_in_tuple(
        [&](auto channel) {
          if (channel.name == topic) {
            using ReSimType = typename decltype(channel)::type;
            ReSimType logged_message;
            EXPECT_TRUE(logged_message.ParseFromArray(
                view.message.data,
                view.message.dataSize));
            auto &expected_message = static_cast<const ReSimType &>(
                *test_messages.at(topic).at(time));

            verify_equality(logged_message, expected_message);
            EXPECT_EQ(type, ReSimType::GetDescriptor()->full_name());

            const std::string expected_data{
                dependency_file_descriptor_set(*ReSimType::GetDescriptor())};
            EXPECT_EQ(expected_data.size(), view.schema->data.size());
            EXPECT_EQ(
                0,
                std::memcmp(
                    expected_data.data(),
                    view.schema->data.data(),
                    expected_data.size()));
          }
          // Need to return something even though we don't use it.
          return true;
        },
        TEST_CHANNELS);
  }
}

}  // namespace

TEST(ResimLogFromRos2Test, TestConversion) {
  // SETUP
  // Make sure we clean up protobuf:
  static bool registered = false;
  if (not registered) {
    std::atexit(google::protobuf::ShutdownProtobufLibrary);
    registered = true;
  }

  const testing::TestDirectoryRAII test_directory;
  const std::filesystem::path input_path{test_directory.test_file_path()};
  const std::filesystem::path output_path{
      test_directory.test_file_path("mcap")};
  constexpr auto PLUGIN_PATH = "resim/msg/default_converter_plugin.so";

  populate_log(input_path);

  // ACTION
  resim_log_from_ros2(PLUGIN_PATH, input_path, output_path);

  // VERIFICATION
  verify_log_contents(output_path);
}

}  // namespace resim::msg
