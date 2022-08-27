#include "resim_core/utils/double_buffer.hh"

#include <gtest/gtest.h>

#include <string>

namespace resim {

namespace {

// An enum class used to help specify test elements using
// specialization.
enum class Index : std::size_t {
  CURR = 0U,
  NEXT,
};

// A variable template to use for test values of various types.
template <typename Type, Index Idx>
constexpr Type TEST_VALUE;

// Partial specializations for all the types we hope to test with.
template <>
constexpr int TEST_VALUE<int, Index::CURR> = 1;
template <>
constexpr int TEST_VALUE<int, Index::NEXT> = 2;
template <>
constexpr double TEST_VALUE<double, Index::CURR> = 1.0;
template <>
constexpr double TEST_VALUE<double, Index::NEXT> = 2.0;
template <>
constexpr char TEST_VALUE<char, Index::CURR> = 'c';
template <>
constexpr char TEST_VALUE<char, Index::NEXT> = 'n';

}  // namespace

template <typename T>
class DoubleBufferTest : public ::testing::Test {};

using TestTypes = ::testing::Types<int, double, char>;
TYPED_TEST_SUITE(DoubleBufferTest, TestTypes);

// Test that we can construct double buffers
TYPED_TEST(DoubleBufferTest, TestConstruction) {
  const DoubleBuffer<TypeParam> double_buffer;

  constexpr TypeParam CURR_VALUE = TEST_VALUE<TypeParam, Index::CURR>;
  constexpr TypeParam NEXT_VALUE = TEST_VALUE<TypeParam, Index::NEXT>;
  const DoubleBuffer<TypeParam> double_buffer_initialized{
      CURR_VALUE,
      NEXT_VALUE};

  EXPECT_EQ(double_buffer.current(), TypeParam{});
  EXPECT_EQ(double_buffer.next(), TypeParam{});

  EXPECT_EQ(double_buffer_initialized.current(), CURR_VALUE);
  EXPECT_EQ(double_buffer_initialized.next(), NEXT_VALUE);
}

// Test that we can successfully modify the next element of the buffer
TYPED_TEST(DoubleBufferTest, TestModification) {
  // SETUP
  constexpr TypeParam CURR_VALUE = TEST_VALUE<TypeParam, Index::CURR>;
  constexpr TypeParam NEXT_VALUE = TEST_VALUE<TypeParam, Index::NEXT>;

  DoubleBuffer<TypeParam> double_buffer{CURR_VALUE, NEXT_VALUE};

  // VERIFICATION
  EXPECT_NE(double_buffer.next(), CURR_VALUE);

  // ACTION
  double_buffer.mutable_next() = CURR_VALUE;

  // VERIFICATION
  EXPECT_EQ(double_buffer.next(), CURR_VALUE);
}

// Test that we can swap the elements in the buffer
TYPED_TEST(DoubleBufferTest, TestSwap) {
  // SETUP
  constexpr TypeParam CURR_VALUE = TEST_VALUE<TypeParam, Index::CURR>;
  constexpr TypeParam NEXT_VALUE = TEST_VALUE<TypeParam, Index::NEXT>;

  DoubleBuffer<TypeParam> double_buffer{CURR_VALUE, NEXT_VALUE};

  // VERIFICATION
  EXPECT_NE(double_buffer.next(), CURR_VALUE);
  EXPECT_NE(double_buffer.current(), NEXT_VALUE);

  // ACTION
  double_buffer.swap();

  // VERIFICATION
  EXPECT_EQ(double_buffer.next(), CURR_VALUE);
  EXPECT_EQ(double_buffer.current(), NEXT_VALUE);
}

}  // namespace resim
