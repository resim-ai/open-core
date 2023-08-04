// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/utils/integer_power.hh"

#include <cmath>
#include <cstdint>
#include <limits>

#include "gtest/gtest.h"
#include "resim/assert/assert.hh"

namespace resim {

template <typename UInt_t>
class IntegerPowerTest : public ::testing::Test {};

template <typename UInt_t>
class IntegerPowerDeathTest : public ::testing::Test {};

using UIntTypes = ::testing::Types<uint8_t, uint16_t, uint32_t, uint64_t>;
TYPED_TEST_SUITE(IntegerPowerTest, UIntTypes);
TYPED_TEST_SUITE(IntegerPowerDeathTest, UIntTypes);

TYPED_TEST(IntegerPowerTest, TestIntegerPowers) {
  constexpr TypeParam MAX_BASE = 5U;
  constexpr TypeParam MAX_EXPONENT = 5U;
  for (TypeParam base = 1U; base < MAX_BASE; ++base) {
    for (TypeParam exponent = 0U; exponent < MAX_EXPONENT; ++exponent) {
      if (std::pow(base, exponent) <
          static_cast<double>(std::numeric_limits<TypeParam>::max())) {
        EXPECT_EQ(pow(base, exponent), std::pow(base, exponent));
      }
    }
  }
}

TYPED_TEST(IntegerPowerTest, TestZeroToZero) { EXPECT_EQ(pow(0U, 0U), 1U); }

TYPED_TEST(IntegerPowerDeathTest, TestOverFlow) {
  // SETUP
  constexpr TypeParam TWO{2U};
  constexpr TypeParam WIDTH{std::numeric_limits<TypeParam>::digits};

  // ACTION / VERIFICATION
  // Should work fine:
  pow(TWO, static_cast<TypeParam>(WIDTH - 1U));
  // Should cause death:
  EXPECT_THROW(pow(TWO, WIDTH), AssertException);
}

}  // namespace resim
