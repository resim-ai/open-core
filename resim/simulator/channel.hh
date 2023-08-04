// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#pragma once

#include <memory>
#include <utility>
#include <vector>

namespace resim::simulator {

// A base class for all channels so we can hold them all in a shared
// ChannelRegistry as we're setting up all the channels by name at the beginning
// of an execution.
class ChannelBase {
 public:
  virtual ~ChannelBase() = default;
  ChannelBase() = default;

  // Delete all auto-generated constructors so users must use
  // Channel<T>::create() (below) to make a Channel.
  ChannelBase(const ChannelBase &) = delete;
  ChannelBase(ChannelBase &&) = delete;
  ChannelBase &operator=(const ChannelBase &) = delete;
  ChannelBase &operator=(ChannelBase &&) = delete;
};

// Forward declaration since we want publishers to be creatable only by the
// Channel object.
template <typename T>
class Publisher;

// This class represents a collection of data of type T that can be written to
// by a collection of Publisher<T> objects that it's responsible for
// creating. When a new Publisher<T> is created, it's given a piece of memory
// that it, and only it, can write to. Therefore, this object always holds one
// entry for every publisher it has ever created. This is intended to be used as
// a communications buffer where many publishers can write into a common pool
// that subscribers have read-only access to through shared ownership of the
// Channel itself. This class inherits from std::enable_shared_from_this<> so
// that new Publishers can share ownership of this Channel in order to write to
// it. Consequently, this class is only constructable through the static
// create() member function which makes a shared pointer.
template <typename T>
class Channel final : public ChannelBase,
                      public std::enable_shared_from_this<Channel<T>> {
 public:
  // Delete all auto-generated constructors so users must use create() to make a
  // Channel. The default constructor is private (below).
  Channel(const Channel<T> &) = delete;
  Channel(Channel<T> &&) = delete;
  Channel<T> &operator=(const Channel<T> &) = delete;
  Channel<T> &operator=(Channel<T> &&) = delete;

  ~Channel() override = default;

  // Static factory. We inherit from std::enable_shared_from_this<>
  // so this object *must* be managed by a shared pointer or undefined
  // behavior will result. Hence we force users to create this object
  // only in this way.
  static std::shared_ptr<Channel<T>> create();

  // The one true way to make a publisher. Adds a new entry in the data_ vector
  // that the new Publisher<T> and only the new Publisher<T> can write to.
  std::unique_ptr<const Publisher<T>> make_publisher();

  // Getter for the data vector.
  const std::vector<T> &data() const;

 private:
  // Publisher<T> needs to be a friend because we only want it to be able to
  // write to data_.
  friend class Publisher<T>;

  // The id used by each publisher to identify the slot it can publish into.
  using SlotId = std::size_t;

  // Private default constructor used only by create() above.
  Channel() = default;

  // The underlying data slots that publishers write to.
  std::vector<T> data_;
};

// This is the Publisher class which holds shared ownership of the Channel that
// created it and which writes to a single slot in that channel, identified by
// the SlotId it is given upon construction.
template <typename T>
class Publisher final {
 public:
  // Delete or make private all constructors so that only Channel<T> can make
  // this class.
  Publisher(const Publisher<T> &) = delete;
  Publisher(Publisher<T> &&) = delete;
  Publisher<T> &operator=(const Publisher<T> &) = delete;
  Publisher<T> &operator=(Publisher<T> &&) = delete;

  ~Publisher() = default;

  // Publish an rvalue into this publisher's slot.
  // @param[in] x - The value to publish.
  void publish(T &&x) const;

  // Publish an lvalue into this publisher's slot.
  // @param[in] x - The value to publish.
  void publish(const T &x) const;

 private:
  // Channel<T> needs to be a friend to create this object.
  friend class Channel<T>;

  // Constructor used by Channel<T> to make this object.
  // @param[in] channel - The channel making this object.
  // @param[in] id - The slot id for this object.
  Publisher(
      std::shared_ptr<Channel<T>> channel,
      typename Channel<T>::SlotId id);

  // The channel that created this object
  std::shared_ptr<Channel<T>> channel_;

  // The id of this object's slot within channel_;
  typename Channel<T>::SlotId id_;
};

template <typename T>
std::shared_ptr<Channel<T>> Channel<T>::create() {
  // Cannot use std::make_shared because Channel<T> is private.
  return std::shared_ptr<Channel<T>>{new Channel<T>};
}

template <typename T>
std::unique_ptr<const Publisher<T>> Channel<T>::make_publisher() {
  // Users can use std::optional if their type doesn't have a default
  // constructor.
  data_.push_back(T{});
  // Cannot use std::make_unique because Publisher<T> is private.
  return std::unique_ptr<const Publisher<T>>{
      new Publisher<T>{Channel<T>::shared_from_this(), data_.size() - 1U}};
}

template <typename T>
const std::vector<T> &Channel<T>::data() const {
  return data_;
}

template <typename T>
void Publisher<T>::publish(T &&x) const {
  channel_->data_.at(id_) = std::move(x);
}

template <typename T>
void Publisher<T>::publish(const T &x) const {
  channel_->data_.at(id_) = x;
}

template <typename T>
Publisher<T>::Publisher(
    std::shared_ptr<Channel<T>> channel,
    typename Channel<T>::SlotId id)
    : channel_{std::move(channel)},
      id_{id} {}

}  // namespace resim::simulator
