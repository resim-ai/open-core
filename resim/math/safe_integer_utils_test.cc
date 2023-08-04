// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/math/safe_integer_utils.hh"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "resim/assert/assert.hh"

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
    EXPECT_THROW(safe_difference(MIN + ii, ii + 1), AssertException);
    EXPECT_THROW(safe_difference(MAX - ii, -ii - 1), AssertException);
    EXPECT_THROW(safe_difference(ii, ii + MIN), AssertException);
    EXPECT_THROW(safe_difference(-ii - 2, MAX - ii), AssertException);
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
    EXPECT_THROW(safe_sum(MIN + ii, -ii - 1), AssertException);
    EXPECT_THROW(safe_sum(MAX - ii, ii + 1), AssertException);
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
  EXPECT_THROW(safe_abs(MIN), AssertException);
}

}  // namespace resim::math
