// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/planning/cost_building_blocks.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>

#include "resim/assert/assert.hh"
#include "resim/math/is_approx.hh"
#include "resim/testing/random_matrix.hh"

namespace resim::planning {

namespace {

// Helper to check that a given VectorCostResult has no diffs in it, since we do
// this multiple times.
template <int DIM>
void expect_has_no_diffs(const VectorCostResult<DIM> &result) {
  ASSERT_FALSE(result.dcost_dx.has_value());
  ASSERT_FALSE(result.d2cost_dx2.has_value());
}

// Helper function which checks whether a given vector cost function has finite
// differences consistent with the reported fist and second derivatives at x.
template <int DIM, typename Cost>
void expect_finite_differences_match(
    Cost &&cost,
    const Eigen::Matrix<double, DIM, 1> &x) {
  using Mat = Eigen::Matrix<double, DIM, DIM>;
  using Vec = Eigen::Matrix<double, DIM, 1>;

  Vec dcost_dx_fd{Vec::Zero()};
  Mat d2cost_dx2_fd{Mat::Zero()};
  constexpr double EPSILON = 5e-6;

  VectorCostResult<DIM> result{cost(x, ComputeDiffs::YES)};
  ASSERT_TRUE(result.dcost_dx.has_value());
  ASSERT_TRUE(result.d2cost_dx2.has_value());

  for (int ii = 0; ii < DIM; ++ii) {
    VectorCostResult<DIM> perturbed_ii{
        cost(Vec{x + EPSILON * Vec::Unit(ii)}, ComputeDiffs::NO)};
    expect_has_no_diffs(perturbed_ii);

    dcost_dx_fd(ii) = (perturbed_ii.cost - result.cost) / EPSILON;
    for (int jj = 0; jj < DIM; ++jj) {
      VectorCostResult<DIM> perturbed_jj{
          cost(Vec{x + EPSILON * Vec::Unit(jj)}, ComputeDiffs::NO)};
      expect_has_no_diffs(perturbed_jj);

      VectorCostResult<DIM> perturbed_ij{cost(
          Vec{x + EPSILON * (Vec::Unit(ii) + Vec::Unit(jj))},
          ComputeDiffs::NO)};
      expect_has_no_diffs(perturbed_ij);

      constexpr double SQUARED = 2.;
      d2cost_dx2_fd(ii, jj) = ((perturbed_ij.cost - perturbed_ii.cost) -
                               (perturbed_jj.cost - result.cost)) /
                              std::pow(EPSILON, SQUARED);
    }
  }

  constexpr double TOLERANCE = 1e-4;
  const double derivative_tolerance =
      TOLERANCE * std::max(1.0, dcost_dx_fd.norm());
  const double second_derivative_tolerance =
      TOLERANCE * std::max(1.0, d2cost_dx2_fd.norm());

  // I check this above with ASSERT
  // NOLINTBEGIN(bugprone-unchecked-optional-access)
  EXPECT_TRUE(
      math::is_approx(*result.dcost_dx, dcost_dx_fd, derivative_tolerance));
  EXPECT_TRUE(math::is_approx(
      *result.d2cost_dx2,
      d2cost_dx2_fd,
      second_derivative_tolerance));
  // NOLINTEND(bugprone-unchecked-optional-access)
}

// Helper to test the quadratic cost once
template <int DIM, typename RNG>
void test_quadratic_cost_once(RNG &&rng) {
  using Mat = Eigen::Matrix<double, DIM, DIM>;
  using Vec = Eigen::Matrix<double, DIM, 1>;

  Mat curvature{testing::random_matrix<Mat>(rng)};
  curvature = curvature * curvature.transpose().eval();

  const Vec x{testing::random_vector<Vec>(rng)};
  expect_finite_differences_match<DIM>(
      [&curvature](const Vec &x, const ComputeDiffs compute_diffs) {
        return quadratic_cost(x, curvature, compute_diffs);
      },
      x);
}

// Helper to test the soft abs cost once
template <int DIM, typename RNG>
void test_soft_abs_cost_once(RNG &&rng) {
  using Mat = Eigen::Matrix<double, DIM, DIM>;
  using Vec = Eigen::Matrix<double, DIM, 1>;

  Mat curvature{testing::random_matrix<Mat>(rng)};
  curvature = curvature * curvature.transpose().eval();

  constexpr double CUTOFF_LB = 0.1;
  constexpr double CUTOFF_UB = 2.0;
  std::uniform_real_distribution<double> dist{CUTOFF_LB, CUTOFF_UB};
  const double cutoff = dist(rng);

  const Vec x{testing::random_vector<Vec>(rng)};
  expect_finite_differences_match<DIM>(
      [&curvature, cutoff](const Vec &x, const ComputeDiffs compute_diffs) {
        return soft_abs_cost(x, curvature, cutoff, compute_diffs);
      },
      x);
}

// Helper to test the avoidance cost once
template <int DIM, typename RNG>
void test_avoidance_cost_once(RNG &&rng) {
  using Mat = Eigen::Matrix<double, DIM, DIM>;
  using Vec = Eigen::Matrix<double, DIM, 1>;

  Mat curvature{testing::random_matrix<Mat>(rng)};
  curvature = curvature * curvature.transpose().eval();

  constexpr double CUTOFF_LB = 0.1;
  constexpr double CUTOFF_UB = 2.0;
  std::uniform_real_distribution<double> dist{CUTOFF_LB, CUTOFF_UB};
  const double cutoff = dist(rng);

  const Vec x{testing::random_vector<Vec>(rng)};
  expect_finite_differences_match<DIM>(
      [&curvature, cutoff](const Vec &x, const ComputeDiffs compute_diffs) {
        return avoidance_cost(x, curvature, cutoff, compute_diffs);
      },
      x);
}

}  // namespace

TEST(CostBuildingBlocksTest, TestQuadraticCost) {
  const unsigned SEED = 27U;
  std::mt19937 rng{SEED};

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    test_quadratic_cost_once<3>(rng);
  }
}

TEST(CostBuildingBlocksTest, TestSoftAbsCost) {
  const unsigned SEED = 3U;
  std::mt19937 rng{SEED};

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    test_soft_abs_cost_once<3>(rng);
  }
}

TEST(CostBuildingBlocksTest, TestAvoidanceCost) {
  const unsigned SEED = 3U;
  std::mt19937 rng{SEED};

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    test_avoidance_cost_once<3>(rng);
  }
}

// Test that the soft abs and avoidance costs fail on zero or negative values of
// the cutoff.
// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(CostBuildingBlocksTest, TestFailOnNegativeOffset) {
  constexpr int DIM = 3;
  using Mat = Eigen::Matrix<double, DIM, DIM>;
  using Vec = Eigen::Matrix<double, DIM, 1>;

  const unsigned SEED = 3U;
  std::mt19937 rng{SEED};

  Mat curvature{testing::random_matrix<Mat>(rng)};
  curvature = curvature * curvature.transpose().eval();
  const Vec x{testing::random_vector<Vec>(rng)};

  for (const double cutoff : {0.0, -0.1}) {
    EXPECT_THROW(
        soft_abs_cost(x, curvature, cutoff, ComputeDiffs::NO),
        AssertException);
    EXPECT_THROW(
        avoidance_cost(x, curvature, cutoff, ComputeDiffs::NO),
        AssertException);
  }
}
// NOLINTEND(readability-function-cognitive-complexity)
}  // namespace resim::planning
