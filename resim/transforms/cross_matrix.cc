
#include "resim/transforms/cross_matrix.hh"

namespace resim::transforms {

Eigen::Matrix3d cross_matrix(const Eigen::Vector3d &vec) {
  constexpr double ZERO = 0;
  Eigen::Matrix3d crm;
  // clang-format off
  crm << ZERO  , -vec(2),  vec(1),
         vec(2),  ZERO  , -vec(0),
        -vec(1),  vec(0),  ZERO;
  // clang-format on
  return crm;
}

}  // namespace resim::transforms
