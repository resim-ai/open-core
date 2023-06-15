
#include "resim/simulator/channel.hh"

#include <gtest/gtest.h>

#include <cmath>

namespace resim::simulator {

namespace {

template <typename T>
using ChannelPtr = std::shared_ptr<Channel<T>>;

template <typename T>
using PublisherPtr = std::shared_ptr<const Publisher<T>>;

template <typename T>
T test_value(int idx = 0);

template <>
int test_value<int>(int idx) {
  return idx;
}

template <>
double test_value<double>(int idx) {
  return std::cos(static_cast<double>(idx));
}

template <>
std::string test_value<std::string>(int idx) {
  // 'R' repeated idx times:
  return std::string(idx, 'R');
}

}  // namespace

template <typename T>
class ChannelTest : public ::testing::Test {};
using TestTypes = ::testing::Types<int, double, std::string>;
TYPED_TEST_SUITE(ChannelTest, TestTypes);

TYPED_TEST(ChannelTest, TestCreate) {
  // SETUP / ACTION
  const ChannelPtr<TypeParam> channel{Channel<TypeParam>::create()};

  // VERIFICATION
  EXPECT_TRUE(channel->data().empty());
}

TYPED_TEST(ChannelTest, TestChannelMakePublisher) {
  // SETUP
  const ChannelPtr<TypeParam> channel{Channel<TypeParam>::create()};

  // ACTION
  const PublisherPtr<TypeParam> publisher = channel->make_publisher();

  // VERIFICATION
  EXPECT_NE(publisher, nullptr);
  EXPECT_EQ(channel->data().size(), 1U);
}

TYPED_TEST(ChannelTest, TestMakeManyPublishers) {
  // SETUP
  const ChannelPtr<TypeParam> channel{Channel<TypeParam>::create()};

  constexpr int NUM_PUBLISHERS = 100;

  // ACTION
  for (int ii = 0; ii < NUM_PUBLISHERS; ++ii) {
    EXPECT_NE(channel->make_publisher(), nullptr);
  }

  // VERIFICATION
  EXPECT_EQ(channel->data().size(), NUM_PUBLISHERS);
}

TYPED_TEST(ChannelTest, TestPublisherPublish) {
  // SETUP
  const ChannelPtr<TypeParam> channel{Channel<TypeParam>::create()};
  const PublisherPtr<TypeParam> publisher = channel->make_publisher();
  ASSERT_NE(publisher, nullptr);

  // ACTION
  const TypeParam value_to_publish = test_value<TypeParam>();
  publisher->publish(value_to_publish);

  // VERIFICATION
  ASSERT_EQ(channel->data().size(), 1U);
  ASSERT_EQ(channel->data().front(), test_value<TypeParam>(0));
}

TYPED_TEST(ChannelTest, TestPublisherMultiPublish) {
  // SETUP
  const ChannelPtr<TypeParam> channel{Channel<TypeParam>::create()};
  const PublisherPtr<TypeParam> publisher_1 = channel->make_publisher();
  const PublisherPtr<TypeParam> publisher_2 = channel->make_publisher();
  ASSERT_NE(publisher_1, nullptr);
  ASSERT_NE(publisher_2, nullptr);

  // ACTION
  publisher_1->publish(test_value<TypeParam>(0));
  publisher_2->publish(test_value<TypeParam>(1));

  // VERIFICATION
  ASSERT_NE(publisher_1, publisher_2);

  const std::vector<TypeParam> &data{channel->data()};
  ASSERT_EQ(data.size(), 2U);
  ASSERT_NE(
      std::find(data.cbegin(), data.cend(), test_value<TypeParam>(0)),
      data.cend());
  ASSERT_NE(
      std::find(data.cbegin(), data.cend(), test_value<TypeParam>(1)),
      data.cend());
}

// Test that we can publish on multiple publishers and collect the results in a
// single channel. This is called repeatedly by the test case below. It is
// factored out into a separate function to keep the test case simple.
template <typename TypeParam>
void test_publisher_repeated_multi_publish_once(
    const ChannelPtr<TypeParam> &channel,
    const PublisherPtr<TypeParam> &publisher_1,
    const PublisherPtr<TypeParam> &publisher_2,
    const int idx) {
  // ACTION
  publisher_1->publish(test_value<TypeParam>(idx));
  publisher_2->publish(test_value<TypeParam>(idx + 1));

  // VERIFICATION
  const std::vector<TypeParam> &data{channel->data()};
  ASSERT_EQ(data.size(), 2U);
  ASSERT_NE(
      std::find(data.cbegin(), data.cend(), test_value<TypeParam>(idx)),
      data.cend());
  ASSERT_NE(
      std::find(data.cbegin(), data.cend(), test_value<TypeParam>(idx + 1)),
      data.cend());
}

TYPED_TEST(ChannelTest, TestPublisherRepeatedMultiPublish) {
  // SETUP
  const ChannelPtr<TypeParam> channel{Channel<TypeParam>::create()};
  const PublisherPtr<TypeParam> publisher_1 = channel->make_publisher();
  const PublisherPtr<TypeParam> publisher_2 = channel->make_publisher();
  ASSERT_NE(publisher_1, nullptr);
  ASSERT_NE(publisher_2, nullptr);
  ASSERT_NE(publisher_1, publisher_2);

  constexpr int NUM_REPEATS = 100;
  for (int ii = 0; ii < NUM_REPEATS; ++ii) {
    test_publisher_repeated_multi_publish_once(
        channel,
        publisher_1,
        publisher_2,
        ii);
  }
}

}  // namespace resim::simulator
