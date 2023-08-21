
#include "resim/msg/fuzz_helpers.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/msg/header.pb.h"
#include "resim/msg/transform.pb.h"

namespace resim::msg {

TEST(FuzzHelpersTest, TestHeaderEqual) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  const Header header{random_element<Header>(InOut{rng})};
  Header header_different_secs{header};
  header_different_secs.mutable_stamp()->set_seconds(
      header.stamp().seconds() + 1);
  Header header_different_nanos{header};
  header_different_nanos.mutable_stamp()->set_nanos(header.stamp().nanos() + 1);
  Header header_different_frame{header};
  header_different_frame.set_frame_id(header.frame_id() + "_different");

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(header, header));
  EXPECT_FALSE(verify_equality(header, header_different_secs));
  EXPECT_FALSE(verify_equality(header, header_different_nanos));
  EXPECT_FALSE(verify_equality(header, header_different_frame));
  EXPECT_FALSE(verify_equality(header_different_secs, header));
  EXPECT_FALSE(verify_equality(header_different_nanos, header));
  EXPECT_FALSE(verify_equality(header_different_frame, header));
}

TEST(FuzzHelpersTest, TestTransformStampedEqual) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  const TransformStamped transform{
      random_element<TransformStamped>(InOut{rng})};

  TransformStamped transform_different_header{transform};
  transform_different_header.mutable_header()->CopyFrom(
      random_element<Header>(InOut{rng}));

  TransformStamped transform_different_child{transform};
  transform_different_child.set_child_frame_id(
      transform.child_frame_id() + "_different");

  TransformStamped transform_different_transform{transform};
  transform_different_transform.mutable_transform()->CopyFrom(
      random_element<TransformStamped>(InOut{rng}).transform());

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(transform, transform));
  EXPECT_FALSE(verify_equality(transform, transform_different_header));
  EXPECT_FALSE(verify_equality(transform, transform_different_child));
  EXPECT_FALSE(verify_equality(transform, transform_different_transform));
  EXPECT_FALSE(verify_equality(transform_different_header, transform));
  EXPECT_FALSE(verify_equality(transform_different_child, transform));
  EXPECT_FALSE(verify_equality(transform_different_transform, transform));
}

TEST(FuzzHelpersTest, TestTransformArrayEqual) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  const TransformArray transform_array{
      random_element<TransformArray>(InOut{rng})};

  TransformArray transform_array_different_size{transform_array};
  ASSERT_GT(transform_array_different_size.transforms_size(), 0U);
  transform_array_different_size.mutable_transforms()->erase(
      transform_array_different_size.transforms().begin());

  TransformArray transform_array_different_element{transform_array};
  ASSERT_GT(transform_array_different_element.transforms_size(), 0U);
  transform_array_different_element.mutable_transforms(0)->CopyFrom(
      random_element<TransformStamped>(InOut{rng}));

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(transform_array, transform_array));
  EXPECT_FALSE(
      verify_equality(transform_array, transform_array_different_size));
  EXPECT_FALSE(
      verify_equality(transform_array, transform_array_different_element));
  EXPECT_FALSE(
      verify_equality(transform_array_different_size, transform_array));
  EXPECT_FALSE(
      verify_equality(transform_array_different_element, transform_array));
}

}  // namespace resim::msg
