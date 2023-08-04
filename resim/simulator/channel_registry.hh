// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#pragma once

#include <glog/logging.h>

#include <string_view>
#include <unordered_map>
#include <utility>

#include "resim/assert/assert.hh"
#include "resim/simulator/channel.hh"

namespace resim::simulator {

// This class maintains a registry of different channels we have created thus
// far. The channels are held in a map by abstract pointer to ChannelBase. When
// a client wants to get a channel, they will know what type that channel should
// be and request it using the channel() getter. If the channel does not already
// exist, it is created with the type given by the client. If it *does* already
// exist, the types are checked for consistency and an exception is thrown since
// each channel has exactly one type. The registry can also be used to get
// publishers for those clients that want to write to a channel. The same
// type-consistency requirement applies and the request for a new publisher is
// satisfied by a newly-created or existing channel.
class ChannelRegistry {
 public:
  // Create a new publisher for a channel on the given topic with the given
  // type. This request is forwarded to a channel on that topic. If no channel
  // currently exists on that topic, a new one is created with the given
  // type. If one does exist, it is used unless the types don't match, in which
  // case an exception is thrown.
  // @param[in] topic - The topic to make a publisher on.
  // @returns a shared pointer to a new publisher on this topic.
  template <typename T>
  std::shared_ptr<const Publisher<T>> make_publisher(
      const std::string_view &topic);

  // Create a new channel or obtain an existing channel on the given topic with
  // the given type. If the given type doesn't match the type of any existing
  // channel on this topic, an exception is thrown. Returns a const shared
  // pointer so that client code can't independently make publishers on this
  // channel. We don't really want clients to add publishers after all channels
  // have been set-up because it could make debugging more complicated.
  // @param[in] topic - The topic to get the channel for.
  // @returns a shared pointer to the created or existing channel.
  template <typename T>
  std::shared_ptr<const Channel<T>> channel(const std::string_view &topic);

  // Reset the channel registry, effectively dumping ownership of all
  // currently-held channels. These may still be owned by other objects in the
  // program.
  void reset();

 private:
  // A simple helper which handles the logic of creating new channels if they
  // don't exist already and checking type consistency if they do.
  // @param[in] topic - The topic to get or make a channel on.
  // @returns a shared pointer to a new or existing channel. Non-const so we can
  //          create publishers.
  template <typename T>
  std::shared_ptr<Channel<T>> get_or_make_channel(
      const std::string_view &topic);

  // The actual map of topics -> channels
  std::unordered_map<std::string_view, std::shared_ptr<ChannelBase>> channels_;
};

template <typename T>
std::shared_ptr<const Channel<T>> ChannelRegistry::channel(
    const std::string_view &topic) {
  return get_or_make_channel<T>(topic);
}

template <typename T>
std::shared_ptr<Channel<T>> ChannelRegistry::get_or_make_channel(
    const std::string_view &topic) {
  if (channels_.contains(topic)) {
    auto concrete_channel{
        std::dynamic_pointer_cast<Channel<T>>(channels_.at(topic))};
    constexpr auto ERROR_MSG =
        "A channel with this topic name already exists, but it has a different "
        "type to the one specified here.";
    REASSERT(concrete_channel != nullptr, ERROR_MSG);
    return std::move(concrete_channel);
  }
  auto channel = Channel<T>::create();
  channels_.emplace(topic, channel);
  return std::move(channel);
}

template <typename T>
std::shared_ptr<const Publisher<T>> ChannelRegistry::make_publisher(
    const std::string_view &topic) {
  return get_or_make_channel<T>(topic)->make_publisher();
}

}  // namespace resim::simulator
