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

// This alias represents a differentiable function from R^N to R^N.
// Example signiture:
//
// using Vec3 = Eigen::Vector3d;
// using Mat3 = Eigen::Matrix3d;
//
// Vec3 my_three_d_function(const Vec3 &x,
//                          NullableReference<Mat3> jacobian);
//
// In this case, the first argument `x` is the intput to the function, and the
// second argument `jacobian` is a mutable matrix which we can populate with the
// Jacobian matrix (df_i(x)/dx_j) of the function. This is a nullable reference
// because not all algorithms which use differentiable functions require their
// derivatives. Such algorithms can simply pass a null_reference<...> object
// whenever they don't require this function to compute them.
template <int DIM>
using DifferentiableFunction = std::function<Eigen::Matrix<double, DIM, 1>(
    const Eigen::Matrix<double, DIM, 1> &,
    NullableReference<Eigen::Matrix<double, DIM, DIM>>)>;

// Use Newton's method (https://en.wikipedia.org/wiki/Newton%27s_method) to
// attempt to compute a root of the given differentiable function.

// @param[in] fun - A differentiable function to find a root of (i.e. an x where
//                  f(x) == 0).

// @param[in] guess - An initial guess of an x such that (fx) == 0.
// @param[in] max_iterations - The maximum number of Newton iterations allowed.
// @param[in] tolerance - The absolute error tolerance for termination in the
//                        domain of fun.
// @returns A value x such that fun(x) == 0 if it can be found, or a bad status
//          if the iteration does not converge.
template <int DIM, typename Derived>
StatusValue<Eigen::Matrix<double, DIM, 1>> newton_solve(
    const DifferentiableFunction<DIM> &fun,
    const Eigen::DenseBase<Derived> &guess,
    const int max_iterations,
    const double tolerance) {
  static_assert(Derived::ColsAtCompileTime == 1);
  static_assert(Derived::RowsAtCompileTime == DIM);
  using Vec = Eigen::Matrix<double, DIM, 1>;
  using Mat = Eigen::Matrix<double, DIM, DIM>;
  Vec result{guess};
  Mat jacobian{Mat::Zero()};
  for (int ii = 0; ii <= max_iterations; ++ii) {
    const Vec error{fun(result, NullableReference{jacobian})};
    Eigen::JacobiSVD<Mat> svd{
        jacobian,
        Eigen::ComputeFullU | Eigen::ComputeFullV};

    const Vec delta{-svd.solve(error)};
    result = result + delta;
    if (delta.norm() < tolerance) {
      return result;
    }
  }
  return MAKE_STATUS("Did not converge in Newton iteration!");
}

// This function is a wrapper for the above which makes it easy to pass in
// lambdas for the function being solved. In particular, the above overload can
// run into issues with template argument deduction in such cases. With this
// overload, you can pass lambdas in directly: e.g.
//
// newton_solve(
//     [&A, &b](const Vec &x, NullableReference<Jac> dydx) -> Vec {
//       if (dydx.has_value()) {
//         *dydx = A;  // Easy peasy
//       }
//       return Vec{A * x + b};
//     },
//     Vec::Ones(),
//     MAX_ITERATIONS,
//     TOLERANCE);
//
// Note that this lambda **should almost certainly** have a trailing return type
// as automatic type deduction with Eigen can cause severe bugs. See
// https://eigen.tuxfamily.org/dox/TopicPitfalls.html for why using auto type
// deduction with Eigen is fraught with danger.
//
// @param[in] fun - A differentiable function to find a root of (i.e. an x where
//                  f(x) == 0).
// @param[in] guess - An initial guess of an x such that (fx) == 0.
// @param[in] max_iterations - The maximum number of Newton iterations allowed.
// @param[in] tolerance - The absolute error tolerance for termination in the
//                        domain of fun.
// @returns A value x such that fun(x) == 0 if it can be found, or a bad status
//          if the iteration does not converge.
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
  // without a explicit trailing return type.
  static_assert(std::is_same_v<
                decltype(fun(
                    std::declval<Vec>(),
                    std::declval<NullableReference<Mat>>())),
                Vec>);
  return newton_solve(
      DifferentiableFunction<DIM>(fun),
      guess,
      max_iterations,
      tolerance);
}

}  // namespace resim::math
