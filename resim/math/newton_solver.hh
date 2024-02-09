// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>

#include "resim/utils/nullable_reference.hh"
#include "resim/utils/status_value.hh"

namespace resim::math {

template <int DIM>
using DifferentiableFunction = std::function<Eigen::Matrix<double, DIM, 1>(
    const Eigen::Matrix<double, DIM, 1> &,
    NullableReference<Eigen::Matrix<double, DIM, DIM>>)>;

template <int DIM>
StatusValue<Eigen::Matrix<double, DIM, 1>> newton_solve(
    const DifferentiableFunction<DIM> &fun,
    const Eigen::Matrix<double, DIM, 1> &guess,
    const int max_iterations,
    const double tolerance) {
  using Vec = Eigen::Matrix<double, DIM, 1>;
  using Mat = Eigen::Matrix<double, DIM, DIM>;
  Vec result{guess};
  Mat jacobian{Mat::Zero()};
  for (int ii = 0; ii <= max_iterations; ++ii) {
    const Vec error{fun(result, NullableReference{jacobian})};
    if (error.norm() < tolerance) {
      return result;
      break;
    }
    Eigen::JacobiSVD<Mat> svd{
        jacobian,
        Eigen::ComputeFullU | Eigen::ComputeFullV};
    result = result - svd.solve(error);
  }
  return MAKE_STATUS("Did not converge in Newton iteration!");
}

template <typename Callable, typename Derived>
StatusValue<Eigen::Matrix<double, Derived::RowsAtCompileTime, 1>> newton_solve(
    const Callable &fun,
    const Eigen::DenseBase<Derived> &guess,
    const int max_iterations,
    const double tolerance) {
  constexpr int DIM = Derived::RowsAtCompileTime;
  static_assert(Derived::ColsAtCompileTime == 1);
  using Vec = Eigen::Matrix<double, DIM, 1>;
  using Mat = Eigen::Matrix<double, DIM, DIM>;

  // Catch at compile time when users are inadvertently returning a
  // proxy type from Eigen. This can easily happen if fun is a lambda
  // without a explicit trailing return type. See
  // https://eigen.tuxfamily.org/dox/TopicPitfalls.html for why using
  // auto type deduction with Eigen is fraught with danger.
  static_assert(std::is_same_v<
                decltype(fun(
                    std::declval<Vec>(),
                    std::declval<NullableReference<Mat>>())),
                Vec>);
  return newton_solve(
      DifferentiableFunction<DIM>(fun),
      Eigen::Matrix<double, DIM, 1>{guess},
      max_iterations,
      tolerance);
}

}  // namespace resim::math
