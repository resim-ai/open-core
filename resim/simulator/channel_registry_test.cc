// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/simulator/channel_registry.hh"

#include <gtest/gtest.h>

#include "resim/assert/assert.hh"

namespace resim::simulator {

template <typename T>
using ChannelPtr = std::shared_ptr<const Channel<T>>;

template <typename T>
using PublisherPtr = std::shared_ptr<const Publisher<T>>;

template <typename T>
class ChannelRegistryTest : public ::testing::Test {};
using TestTypes = ::testing::Types<int, double>;
TYPED_TEST_SUITE(ChannelRegistryTest, TestTypes);

TYPED_TEST(ChannelRegistryTest, TestChannelGetter) {
  // SETUP
  ChannelRegistry registry;
  constexpr std::string_view TOPIC = "MY_TOPIC";
  constexpr std::string_view OTHER_TOPIC = "OTHER_TOPIC";

  // ACTION
  const ChannelPtr<TypeParam> channel_a{registry.channel<TypeParam>(TOPIC)};
  const ChannelPtr<TypeParam> channel_b{registry.channel<TypeParam>(TOPIC)};
  const ChannelPtr<TypeParam> channel_c{
      registry.channel<TypeParam>(OTHER_TOPIC)};

  // VERIFICATION
  // We should get the same channel for the same topic.
  EXPECT_EQ(channel_a, channel_b);

  // We should get a different channel for a different topic.
  EXPECT_NE(channel_a, channel_c);
}

TYPED_TEST(ChannelRegistryTest, TestReset) {
  // SETUP
  ChannelRegistry registry;
  constexpr std::string_view TOPIC = "MY_TOPIC";

  // ACTION
  const ChannelPtr<TypeParam> channel_a{registry.channel<TypeParam>(TOPIC)};
  registry.reset();
  const ChannelPtr<TypeParam> channel_b{registry.channel<TypeParam>(TOPIC)};

  // VERIFICATION
  // We should get a new channel for the same topic since we reset.
  EXPECT_NE(channel_a, channel_b);
}

TYPED_TEST(ChannelRegistryTest, TestMakePublisher) {
  // SETUP
  ChannelRegistry registry;
  constexpr std::string_view TOPIC = "MY_TOPIC";
  const TypeParam MY_VALUE = 3;

  const ChannelPtr<TypeParam> channel{registry.channel<TypeParam>(TOPIC)};

  // ACTION
  const PublisherPtr<TypeParam> publisher{
      registry.make_publisher<TypeParam>(TOPIC)};

  // VERIFICATION
  // Make sure the publisher is connected to the channel.
  publisher->publish(MY_VALUE);
  EXPECT_EQ(channel->data().size(), 1U);
  EXPECT_EQ(channel->data().front(), MY_VALUE);
}

TEST(ChannelRegistryDeathTest, TestChannelChannelBadType) {
  // SETUP
  ChannelRegistry registry;

  constexpr std::string_view TOPIC = "MY_TOPIC";
  using OneType = int;
  using AnotherType = double;

  // ACTION / VERIFICATION
  registry.channel<OneType>(TOPIC);
  EXPECT_THROW(registry.channel<AnotherType>(TOPIC), AssertException);
}

TEST(ChannelRegistryDeathTest, TestChannelPublisherBadType) {
  // SETUP
  ChannelRegistry registry;

  constexpr std::string_view TOPIC = "MY_TOPIC";
  using OneType = int;
  using AnotherType = double;

  // ACTION / VERIFICATION
  registry.channel<OneType>(TOPIC);
  EXPECT_THROW(registry.make_publisher<AnotherType>(TOPIC), AssertException);
}

TEST(ChannelRegistryDeathTest, TestPublisherChannelBadType) {
  // SETUP
  ChannelRegistry registry;

  constexpr std::string_view TOPIC = "MY_TOPIC";
  using OneType = int;
  using AnotherType = double;

  // ACTION / VERIFICATION
  registry.make_publisher<OneType>(TOPIC);
  EXPECT_THROW(registry.channel<AnotherType>(TOPIC), AssertException);
}

TEST(ChannelRegistryDeathTest, TestPublisherPublisherBadType) {
  // SETUP
  ChannelRegistry registry;

  constexpr std::string_view TOPIC = "MY_TOPIC";
  using OneType = int;
  using AnotherType = double;

  // ACTION / VERIFICATION
  registry.make_publisher<OneType>(TOPIC);
  EXPECT_THROW(registry.make_publisher<AnotherType>(TOPIC), AssertException);
}

}  // namespace resim::simulator
