// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>
#include <functional>
#include <optional>

namespace resim::geometry {

template <int DIM>
using SupportFunction = std::function<Eigen::Matrix<double, DIM, 1>(
    const Eigen::Matrix<double, DIM, 1> &)>;

constexpr int DEFAULT_MAX_ITERATIONS = 100;

// Use the GJK algorithm to determine the minimum distance between the convex
// sets described by support function 1 and support function 2.
// @param support_1 - The support for object 1.
// @param support_2 - The support for object 2.
// @param max_iterations - The max number of iterations to allow for
//                         convergence.
// @returns The minimum distance between the sets, or a nullopt otherwise.
template <int DIM>
std::optional<double> gjk_algorithm(
    const SupportFunction<DIM> &support_1,
    const SupportFunction<DIM> &support_2,
    int max_iterations = DEFAULT_MAX_ITERATIONS);

}  // namespace resim::geometry
