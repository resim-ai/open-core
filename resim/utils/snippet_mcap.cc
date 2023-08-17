
#include "resim/utils/snippet_mcap.hh"

#include <cstdint>
#include <mcap/types.hpp>
#include <unordered_map>

#include "resim/assert/assert.hh"

namespace resim {

void snippet_mcap(
    time::Timestamp start_time,
    time::Timestamp end_time,
    InOut<mcap::McapReader> input_mcap,
    InOut<mcap::McapWriter> output_mcap) {
  std::unordered_map<mcap::SchemaId, mcap::SchemaId> old_to_new_schema_map_;
  std::unordered_map<mcap::ChannelId, mcap::ChannelId> old_to_new_channel_map_;

  const int64_t start_time_count = start_time.time_since_epoch().count();
  const int64_t end_time_count = end_time.time_since_epoch().count();
  REASSERT(
      start_time_count < end_time_count,
      "End time must be later than start time!");
  REASSERT(start_time_count >= 0, "Start time must be after epoch!");

  for (const auto &view : input_mcap->readMessages(
           static_cast<mcap::Timestamp>(start_time_count),
           static_cast<mcap::Timestamp>(end_time_count))) {
    const mcap::SchemaId old_schema_id = view.schema->id;
    const mcap::ChannelId old_channel_id = view.channel->id;
    if (not old_to_new_schema_map_.contains(old_schema_id)) {
      mcap::Schema schema = *view.schema;
      output_mcap->addSchema(schema);
      old_to_new_schema_map_.emplace(old_schema_id, schema.id);
    }
    if (not old_to_new_channel_map_.contains(old_channel_id)) {
      mcap::Channel channel = *view.channel;
      channel.schemaId = old_to_new_schema_map_.at(old_schema_id);
      output_mcap->addChannel(channel);
      old_to_new_channel_map_.emplace(old_channel_id, channel.id);
    }
    mcap::Message message{view.message};
    message.channelId = old_to_new_channel_map_.at(old_channel_id);
    REASSERT(output_mcap->write(message).ok());
  }
}

}  // namespace resim
