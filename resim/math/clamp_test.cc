// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// NOLINTBEGIN(readability-magic-numbers)

#include "resim/math/clamp.hh"

#include <gtest/gtest.h>

#include "resim/assert/assert.hh"

namespace resim::math {

TEST(ClampTest, TestClamp) {
  constexpr double LB = 3.2;
  constexpr double UB = 5.1;
  EXPECT_EQ(4.7, clamp(4.7, LB, UB));
  EXPECT_EQ(LB, clamp(LB - 1.0, LB, UB));
  EXPECT_EQ(UB, clamp(UB + 1.0, LB, UB));
  EXPECT_THROW(clamp(UB + 1.0, UB, LB), AssertException);
}

}  // namespace resim::math

// NOLINTEND(readability-magic-numbers)
