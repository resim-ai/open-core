// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/resim_log_from_ros2.hh"

#include <mcap/writer.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>

#include "resim/assert/assert.hh"
#include "resim/msg/converter_plugin.hh"
#include "resim/utils/inout.hh"

namespace resim::msg {
namespace {

// Map types that we need to track topics, types, schemas, and channels.
using StringStringMap = std::unordered_map<std::string, std::string>;
using StringSchemaMap = std::unordered_map<std::string, mcap::SchemaId>;
using StringChannelMap = std::unordered_map<std::string, mcap::ChannelId>;

// Helper function to add a schema to the given McapWriter.
mcap::SchemaId add_schema(
    const ConverterPlugin::SchemaInfo &schema_info,
    InOut<mcap::McapWriter> writer) {
  mcap::Schema schema{schema_info.name, schema_info.encoding, schema_info.data};

  writer->addSchema(schema);
  return schema.id;
}

constexpr auto PROFILE = "resim_mcap";

}  // namespace

void resim_log_from_ros2(
    const std::filesystem::path &plugin_path,
    const std::filesystem::path &input_log,
    const std::filesystem::path &output_log) {
  ConverterPlugin plugin{plugin_path.string()};
  rosbag2_cpp::Reader reader;

  reader.open(input_log.string());

  mcap::McapWriter writer;
  const mcap::McapWriterOptions options{PROFILE};
  REASSERT(writer.open(output_log.string(), options).ok());

  // Map of topics to data types as strings.
  StringStringMap topic_to_type_map;

  // Map of input data types to corresponding mcap schema ids for the output
  // log.
  StringSchemaMap type_to_schemas_map;

  // Map of topics to mcap channel ids for the output log.
  StringChannelMap topic_to_channels_map;

  // Read through the topics and types to populate the above maps.
  for (const auto &t : reader.get_all_topics_and_types()) {
    if (not plugin.supports_type(t.type)) {
      continue;
    }

    REASSERT(topic_to_type_map.emplace(t.name, t.type).second);
    if (not type_to_schemas_map.contains(t.type)) {
      type_to_schemas_map.emplace(
          t.type,
          add_schema(plugin.get_schema(t.type), InOut{writer}));
    }

    mcap::Channel channel{t.name, "protobuf", type_to_schemas_map.at(t.type)};
    writer.addChannel(channel);
    topic_to_channels_map.emplace(t.name, channel.id);
  }

  // Read the messages and convert them with the plugin.
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    const std::string topic_name = bag_message->topic_name;
    if (not topic_to_type_map.contains(topic_name)) {
      continue;
    }
    const std::string &type = topic_to_type_map.at(topic_name);

    const rclcpp::SerializedMessage extracted_serialized_message(
        *bag_message->serialized_data);
    auto converted = plugin.convert(type, extracted_serialized_message);
    mcap::Message message;
    message.channelId = topic_to_channels_map.at(topic_name);
    message.logTime = bag_message->time_stamp;
    message.publishTime = bag_message->time_stamp;
    message.dataSize = converted.size();
    message.data = converted.data();
    REASSERT(writer.write(message).ok(), "Failed to write message!");
  }
}

}  // namespace resim::msg
