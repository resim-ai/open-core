
#include "resim/testing/fuzz_helpers.hh"

#include <cmath>

namespace resim {

bool verify_equality(double a, double b) {
  constexpr double TOLERANCE = 1e-12;
  return std::fabs(b - a) < TOLERANCE;
}

}  // namespace resim
