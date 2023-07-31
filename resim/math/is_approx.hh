
#pragma once

#include <Eigen/Dense>

namespace resim::math {

constexpr double DEFAULT_PRECISION = 1e-12;

// A helper for telling whether two matrices or vectors are approximately the
// same. This function treats the precision as relative when a and b are large
// and absolute otherwise.
// @param[in] a - The first matrix/vector to compare.
// @param[in] b - The second matrix/vector to compare.
// @param[in] precision - The precision to use in the fuzzy comparison.
template <typename Derived>
bool is_approx(
    const Eigen::MatrixBase<Derived> &a,
    const Eigen::MatrixBase<Derived> &b,
    const double precision = DEFAULT_PRECISION) {
  return (a - b).norm() <=
         precision * std::max(1.0, std::min(a.norm(), b.norm()));
}

}  // namespace resim::math
