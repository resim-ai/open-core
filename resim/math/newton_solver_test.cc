// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/math/newton_solver.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/math/is_approx.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::math {
// NOLINTBEGIN(readability-magic-numbers)

template <int DIM, typename Rng>
void test_linear_solve(Rng &&rng) {
  // SETUP
  using Vec = Eigen::Matrix<double, DIM, 1>;
  using Jac = Eigen::Matrix<double, DIM, DIM>;
  const Jac A{testing::random_matrix<Jac>(rng)};
  const Vec b{testing::random_matrix<Vec>(rng)};

  constexpr int MAX_ITERATIONS = 1;
  constexpr double TOLERANCE = 1e-10;
  // ACTION
  const auto solution_sv = newton_solve(
      [&A, &b](const Vec &x, NullableReference<Jac> dydx) -> Vec {
        if (dydx.has_value()) {
          *dydx = A;  // Easy peasy
        }
        return Vec{A * x + b};
      },
      Vec::Ones(),
      MAX_ITERATIONS,
      TOLERANCE);

  // VERIFICATION
  ASSERT_TRUE(solution_sv.ok());
  EXPECT_TRUE(math::is_approx(solution_sv.value(), -A.inverse() * b));
}

template <int DIM, typename Rng>
void test_nonlinear_solve(Rng &&rng) {
  // SETUP
  using Vec = Eigen::Matrix<double, DIM, 1>;
  using Jac = Eigen::Matrix<double, DIM, DIM>;
  const Jac A{testing::random_matrix<Jac>(rng)};
  const Vec b{testing::random_matrix<Vec>(rng)};

  constexpr double TOLERANCE = 1e-10;
  // ACTION
  const auto fun = [&A, &b](const Vec &x, NullableReference<Jac> dydx) -> Vec {
    if (dydx.has_value()) {
      // d/dx_k [ (sum_j[A_ij x_j])^3 ] = 3 (sum_j[A_ij x_j])^2 * A_ik
      *dydx = 3.0 *
              ((A * (x - b)).array().pow(2).matrix() * Vec::Ones().transpose())
                  .array() *
              A.array();
    }
    return (A * (x - b)).array().pow(3).matrix();
  };

  constexpr int MAX_ITERATIONS = 20;
  const auto solution_sv =
      newton_solve(fun, b + 0.05 * Vec::Ones(), MAX_ITERATIONS, TOLERANCE);

  // VERIFICATION
  ASSERT_TRUE(solution_sv.ok());
  EXPECT_LT(fun(solution_sv.value(), null_reference<Jac>).norm(), TOLERANCE);

  constexpr int TOO_FEW_ITERATIONS = 3;
  const auto unconverged_sv =
      newton_solve(fun, 100.0 * Vec::Ones(), TOO_FEW_ITERATIONS, TOLERANCE);
  EXPECT_FALSE(unconverged_sv.ok());
}

TEST(NewtonSolver, TestSolveLinear) {
  constexpr size_t SEED = 493U;
  std::mt19937 rng{SEED};
  constexpr int NUM_TESTS = 20;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    test_linear_solve<1>(rng);
    test_linear_solve<2>(rng);
    test_linear_solve<3>(rng);
    test_linear_solve<4>(rng);
    test_linear_solve<5>(rng);
    test_linear_solve<6>(rng);
    test_linear_solve<7>(rng);
    test_linear_solve<8>(rng);
  }
}

TEST(NewtonSolver, TestSolveNonLinear) {
  constexpr size_t SEED = 493U;
  std::mt19937 rng{SEED};
  constexpr int NUM_TESTS = 20;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    test_nonlinear_solve<1>(rng);
    test_nonlinear_solve<2>(rng);
    test_nonlinear_solve<3>(rng);
    test_nonlinear_solve<4>(rng);
    test_nonlinear_solve<5>(rng);
    test_nonlinear_solve<6>(rng);
    test_nonlinear_solve<7>(rng);
    test_nonlinear_solve<8>(rng);
  }
}

// NOLINTEND(readability-magic-numbers)
}  // namespace resim::math
