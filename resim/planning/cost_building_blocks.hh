#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <optional>

#include "resim/assert/assert.hh"
#include "resim/utils/nullable_reference.hh"

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
    const Eigen::Matrix<double, DIM, DIM> &curvature,
    const double a,
    const ComputeDiffs compute_diffs) {
  const auto quad = quadratic_cost(x, curvature, compute_diffs);
  REASSERT(a > 0., "a must be strictly positive!");
  REASSERT(quad.cost >= 0., "Quadratic cost must be non-negative!");

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

// An avoidance cost function of the form:
// g(x) = 1 / (0.5 x^T K x + a * a)
// For a DIM x DIM matrix K and a scalar a.
// This function gets largest near zero and drops off as you move further away.
// The magnitude of a and the eigenvalues of K will generally be small so that
// the cost gets very large near zero. a can be interpreted as controlling the
// vertical scaling of the curve where a larger value makes the curve shorter.
// The curvature matrix helps determine the width of the resulting hump in cost,
// where smaller eigenvalues make a larger hump.
// @param[in] x - The vector argument of the cost function.
// @param[in] curvature - A DIM x DIM matrix representing the curvature of the
//                        quadratic part of the cost.
// @param[in] a - The height factor as described above. Do not make a very small
//                or numerical accuracy may suffer near ||x|| == 0.
// @param[in] compute_diffs - A flag telling us whether to compute the
//                            differentials.
template <int DIM>
VectorCostResult<DIM> avoidance_cost(
    const Eigen::Matrix<double, DIM, 1> &x,
    const Eigen::Matrix<double, DIM, DIM> &curvature,
    const double a,
    const ComputeDiffs compute_diffs) {
  const auto quad = quadratic_cost(x, curvature, compute_diffs);
  REASSERT(a > 0., "a must be strictly positive!");
  REASSERT(quad.cost >= 0., "Quadratic cost must be non-negative!");

  VectorCostResult<DIM> result;
  result.cost = 1. / (quad.cost + a * a);
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
    const double dcost_dquad = -1. / std::pow(quad.cost + a * a, 2.);
    const double d2cost_dquad2 = 2. / std::pow(quad.cost + a * a, 3.);
    // These can be numerically inaccurate if cost.quad and a are both small:
    result.dcost_dx = dcost_dquad * dquad_dx;
    result.d2cost_dx2 = dcost_dquad * d2quad_dx2 +
                        d2cost_dquad2 * (dquad_dx * dquad_dx.transpose());
    // NOLINTEND(readability-magic-numbers)
  }
  return result;
}

}  // namespace resim::planning
