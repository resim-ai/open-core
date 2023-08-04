// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/utils/integer_power.hh"

#include <cstdint>
#include <limits>

#include "resim/assert/assert.hh"

namespace resim {

namespace {

// A simple helper used to check if overflow will occur when multiplying a and
// b.
template <typename IntegerType>
void overflow_check(IntegerType a, IntegerType b) requires
    std::is_unsigned_v<IntegerType> {
  constexpr IntegerType MAX_VALUE = std::numeric_limits<IntegerType>::max();
  REASSERT(a <= MAX_VALUE / b, "Overflow detected!");
}

}  // namespace

// An integer version of power using repeated squaring.
template <typename IntegerType>
IntegerType pow(IntegerType base, IntegerType exponent) requires
    std::is_unsigned_v<IntegerType> {
  constexpr IntegerType ONE = 0x1;
  constexpr IntegerType ZERO = 0x0;

  IntegerType power_of_base{base};
  IntegerType result = exponent bitand ONE ? base : ONE;
  for (IntegerType ii = ONE; exponent >> ii != ZERO; ++ii) {
    // Square the base for the current exponent bit.
    overflow_check(power_of_base, power_of_base);
    power_of_base *= power_of_base;

    // If the current exponent bit is non-zero, multiply the repeated-squared
    // base onto the result.
    if (exponent bitand ONE << ii) {
      overflow_check(result, power_of_base);
      result *= power_of_base;
    }
  }

  return result;
}

template uint8_t pow(uint8_t, uint8_t);
template uint16_t pow(uint16_t, uint16_t);
template uint32_t pow(uint32_t, uint32_t);
template uint64_t pow(uint64_t, uint64_t);

}  // namespace resim
