
#pragma once

#include <cstdint>

namespace resim::math {

// Take the difference of two signed integers while checking for overflow or
// underflow.
// @param[in] a - The minuend.
// @param[in] b - The subtrahend.
// @returns The difference.
// @throws if an overflow or underflow is encountered.
int64_t safe_difference(int64_t a, int64_t b);

// Take the sum of two signed integers while checking for overflow or underflow.
// @param[in] a - The first addend.
// @param[in] b - The second addend.
// @returns The sum.
// @throws if an overflow or underflow is encountered.
int64_t safe_sum(int64_t a, int64_t b);

// Take the absolute value of the given signed integer while checking for the
// overflow than can occur if a is the minimum integer representable by this
// signed integer type.
// @param[in] a - The number to take the absolute value of.
// @returns The absolute value.
// @throws if an overflow is encountered.
int64_t safe_abs(int64_t a);

// The message for an underflow.
constexpr auto INT_UNDERFLOW_MSG = "Integer Underflow!";

// The message for an overflow.
constexpr auto INT_OVERFLOW_MSG = "Integer Overflow!";

}  // namespace resim::math
