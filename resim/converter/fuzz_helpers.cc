#include "resim/converter/fuzz_helpers.hh"

#include <cmath>

#include "resim/math/is_approx.hh"

namespace resim::converter {

bool verify_equality(ADLTag /*unused*/, double a, double b) {
  return math::is_approx(a, b);
}

bool verify_equality(
    ADLTag /*unused*/,
    const google::protobuf::Timestamp &a,
    const google::protobuf::Timestamp &b) {
  return a.seconds() == b.seconds() and a.nanos() == b.nanos();
}

}  // namespace resim::converter
