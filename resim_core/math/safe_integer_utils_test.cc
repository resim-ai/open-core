#include "resim_core/math/safe_integer_utils.hh"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace resim::math {

namespace {
constexpr int64_t NUM_POINTS = 100;
constexpr int64_t MIN = std::numeric_limits<int64_t>::min();
constexpr int64_t MAX = std::numeric_limits<int64_t>::max();
}  // namespace

TEST(SafeIntegerUtilsTest, TestSafeDifference) {
  // SETUP
  for (int64_t ii = 0; ii < NUM_POINTS; ++ii) {
    // ACTION / VERIFICATION
    // These should be right up against the boundary of what's representable
    EXPECT_EQ(safe_difference(MIN + ii, ii), MIN);
    EXPECT_EQ(safe_difference(MAX - ii, -ii), MAX);
    EXPECT_EQ(safe_difference(ii - 1, ii + MIN), MAX);
    EXPECT_EQ(safe_difference(-ii - 1, MAX - ii), MIN);
  }
}

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(SafeIntegerUtilsDeathTest, TestSafeDifferenceFails) {
  // SETUP
  for (int64_t ii = 0; ii < NUM_POINTS; ++ii) {
    // ACTION / VERIFICATION
    // These should be just outside the boundary of what's representable
    EXPECT_DEATH(safe_difference(MIN + ii, ii + 1), INT_UNDERFLOW_MSG);
    EXPECT_DEATH(safe_difference(MAX - ii, -ii - 1), INT_OVERFLOW_MSG);
    EXPECT_DEATH(safe_difference(ii, ii + MIN), INT_OVERFLOW_MSG);
    EXPECT_DEATH(safe_difference(-ii - 2, MAX - ii), INT_UNDERFLOW_MSG);
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

TEST(SafeIntegerUtilsTest, TestSafeAddition) {
  // SETUP
  for (int64_t ii = 0; ii < NUM_POINTS; ++ii) {
    // ACTION / VERIFICATION
    // These should be right up against the boundary of what's representable
    EXPECT_EQ(safe_sum(MIN + ii, -ii), MIN);
    EXPECT_EQ(safe_sum(MAX - ii, ii), MAX);
  }
}

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(SafeIntegerUtilsDeathTest, TestSafeAdditionFails) {
  // SETUP
  for (int64_t ii = 0; ii < NUM_POINTS; ++ii) {
    // ACTION / VERIFICATION
    // These should be just outside the boundary of what's representable
    EXPECT_DEATH(safe_sum(MIN + ii, -ii - 1), INT_UNDERFLOW_MSG);
    EXPECT_DEATH(safe_sum(MAX - ii, ii + 1), INT_OVERFLOW_MSG);
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

TEST(SafeIntegerUtilsTest, TestSafeAbs) {
  // SETUP
  for (int64_t ii = 0; ii < NUM_POINTS; ++ii) {
    // ACTION / VERIFICATION
    // These should be right up against the boundary of what's representable
    EXPECT_EQ(safe_abs(MIN + ii + 1), -(MIN + ii + 1));
    EXPECT_EQ(safe_abs(MAX - ii), MAX - ii);
  }
}

TEST(SafeIntegerUtilsDeathTest, TestSafeAbsFails) {
  // abs(MIN) can't be represented by the same signed int type.
  EXPECT_DEATH(safe_abs(MIN), INT_OVERFLOW_MSG);
}

}  // namespace resim::math
