// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>
#include <random>
#include <utility>

namespace resim::testing {

// This function template generates psuedo-random matrices with independent
// entries for testing purposes.
// @tparam[in] Matrix_t - The matrix type to generate. Assumed to be an Eigen
//                        type.
// @param[in] generator - The pseudo random number generator to use
//                        (e.g. std::mt19937).
// @param[in] distribution - The distribution to use
//                           (e.g. std::uniform_real_distribution).
template <
    typename Matrix_t,
    typename RandomGenerator_t,
    typename Distribution_t>
Matrix_t random_matrix(
    RandomGenerator_t &&generator,
    Distribution_t &&distribution) {
  return Matrix_t::NullaryExpr([&]() {
    return std::forward<Distribution_t>(distribution)(
        std::forward<RandomGenerator_t>(generator));
  });
}

// Overload of the above which defaults to uniform real distribution between
// -1.0 and 1.0.
// @tparam[in] Matrix_t - The matrix type to generate. Assumed to be an Eigen
//                        type.
// @param[in] generator - The pseudo random number generator to use
//                        (e.g. std::mt19937).
template <typename Matrix_t, typename RandomGenerator_t>
Matrix_t random_matrix(RandomGenerator_t &&generator) {
  using Scalar = typename Matrix_t::Scalar;
  using Distribution = std::uniform_real_distribution<Scalar>;
  constexpr Scalar LB = -1.0;
  constexpr Scalar UB = 1.0;
  Distribution dist{LB, UB};
  return random_matrix<Matrix_t>(generator, dist);
}

// Wrapper for the above named specifically for use with vector types.
// @tpararm[in] Vector_t - The vector type to generate. Assumed to be an Eigen
//                         type.
// @param[in] args - Arguments to forward to either of the above random_matrix()
//                   functions.
template <typename Vector_t, typename... Args>
Vector_t random_vector(Args &&...args) {
  return random_matrix<Vector_t>(std::forward<Args>(args)...);
}

// A simple helper to generate random quaternions. Note that these are *NOT*
// uniformly distributed orientations.
// TODO(https://app.asana.com/0/1202178773526279/1203262688903982/f)
template <typename Rng>
Eigen::Quaterniond random_quaternion(Rng &&rng) {
  return Eigen::Quaterniond{random_vector<Eigen::Vector4d>(rng).normalized()};
}

}  // namespace resim::testing
