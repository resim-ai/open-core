
#include "resim/msg/fuzz_helpers.hh"

namespace resim::msg {

bool verify_equality(const Header &a, const Header &b) {
  return a.stamp().seconds() == b.stamp().seconds() &&
         a.stamp().nanos() == b.stamp().nanos() && a.frame_id() == b.frame_id();
}

bool verify_equality(const TransformStamped &a, const TransformStamped &b) {
  const bool transforms_equal =
      unpack(a.transform()).is_approx(unpack(b.transform()));
  return transforms_equal && verify_equality(a.header(), b.header()) &&
         a.child_frame_id() == b.child_frame_id();
}

bool verify_equality(const TransformArray &a, const TransformArray &b) {
  if (a.transforms_size() != b.transforms_size()) {
    return false;
  }
  for (int ii = 0; ii < a.transforms_size(); ++ii) {
    if (not verify_equality(a.transforms(ii), b.transforms(ii))) {
      return false;
    }
  }
  return true;
}

}  // namespace resim::msg
