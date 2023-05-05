#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <optional>

#include "resim_core/assert/assert.hh"
#include "resim_core/utils/nullable_reference.hh"

namespace resim::planning {

// This class represents the output(s) of a vector cost function (i.e. a
// function that takes a vector and outputs a scalar cost). This includes the
// cost itself and optionally the first and second derivatives of the function.
template <int DIM>
struct VectorCostResult {
  double cost = 0.0;
  std::optional<Eigen::Matrix<double, DIM, 1>> dcost_dx;
  std::optional<Eigen::Matrix<double, DIM, DIM>> d2cost_dx2;
};

// A flag for whether or not a given cost function should compute diffs.
enum class ComputeDiffs {
  YES,
  NO,
};

// A quadratic cost function of the form:
// g(x) = 0.5 x^T [curvature] x
// @param[in] x - The vector argument of the cost function.
// @param[in] curvature - A DIM x DIM matrix representing the curvature of the
//                        cost.
// @param[in] compute_diffs - A flag telling us whether to compute the
//                            differentials.
template <int DIM>
VectorCostResult<DIM> quadratic_cost(
    const Eigen::Matrix<double, DIM, 1> &x,
    const Eigen::Matrix<double, DIM, DIM> &curvature,
    const ComputeDiffs compute_diffs) {
  VectorCostResult<DIM> result;
  constexpr double HALF = 0.5;
  result.cost = HALF * x.dot(curvature * x);

  if (compute_diffs == ComputeDiffs::YES) {
    result.dcost_dx = curvature * x;
    result.d2cost_dx2 = curvature;
  }
  return result;
}

// A soft absolute value cost function of the form:
// g(x) = sqrt(0.5 (x^T [curvature] x) + a * a) - a
// This function behaves like |0.5 x^T [curvature] x| for large values of |x|.
// Note that curvature must be positive definite for this cost function to work
// in general.
// @param[in] x - The vector argument of the cost function.
// @param[in] curvature - A DIM x DIM matrix representing the curvature of the
//                        quadratic part of the cost.
// @param[in] a - The cutoff factor determining roughly where the
//                absolute-value-like behavior begins to dominate.
// @param[in] compute_diffs - A flag telling us whether to compute the
//                            differentials.
template <int DIM>
VectorCostResult<DIM> soft_abs_cost(
    const Eigen::Matrix<double, DIM, 1> &x,
    Eigen::Matrix<double, DIM, DIM> curvature,
    double a,
    const ComputeDiffs compute_diffs) {
  const auto quad = quadratic_cost(x, curvature, compute_diffs);

  VectorCostResult<DIM> result;
  result.cost = std::sqrt(quad.cost + a * a) - a;
  if (compute_diffs == ComputeDiffs::YES) {
    REASSERT(quad.dcost_dx.has_value());
    REASSERT(quad.d2cost_dx2.has_value());

    // We just checked this
    // NOLINTBEGIN(bugprone-unchecked-optional-access)
    const Eigen::Matrix<double, DIM, 1> dquad_dx{*quad.dcost_dx};
    const Eigen::Matrix<double, DIM, DIM> d2quad_dx2{*quad.d2cost_dx2};
    // NOLINTEND(bugprone-unchecked-optional-access)

    // These are mathematical expressions so the magic numbers / coefficients
    // are fine.
    // NOLINTBEGIN(readability-magic-numbers)
    result.dcost_dx = (1. / std::sqrt(quad.cost + a * a) / 2.) * dquad_dx;
    result.d2cost_dx2 = (1. / std::sqrt(quad.cost + a * a) / 2.) * d2quad_dx2 -
                        (std::pow(quad.cost + a * a, -1.5) / 4.) *
                            (dquad_dx * dquad_dx.transpose());
    // NOLINTEND(readability-magic-numbers)
  }
  return result;
}

}  // namespace resim::planning
