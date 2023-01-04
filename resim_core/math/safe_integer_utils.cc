
#include "resim_core/math/safe_integer_utils.hh"

#include <glog/logging.h>

#include <limits>

namespace resim::math {

int64_t safe_difference(const int64_t a, const int64_t b) {
  // There's only one case where we can't negate b and that's when it's the
  // minimum integer since every other integer has an additive inverse.
  if (b != std::numeric_limits<int64_t>::min()) {
    return safe_sum(a, -b);
  }
  // In this case, it's guaranteed to work if a is strictly less than zero since
  // the result will be less than the maximum integer.
  CHECK(a < 0) << INT_OVERFLOW_MSG;
  return a - b;
}

int64_t safe_sum(int64_t a, int64_t b) {
  constexpr int64_t MAX = std::numeric_limits<int64_t>::max();
  constexpr int64_t MIN = std::numeric_limits<int64_t>::min();

  if (a > 0) {
    CHECK(MAX - a >= b) << INT_OVERFLOW_MSG;
  } else {
    CHECK(MIN - a <= b) << INT_UNDERFLOW_MSG;
  }
  return a + b;
}

int64_t safe_abs(int64_t a) {
  // There's only one signed integer whose abs can't be represented:
  CHECK(a != std::numeric_limits<int64_t>::min()) << INT_OVERFLOW_MSG;
  return a > 0 ? a : -a;
}

}  // namespace resim::math
