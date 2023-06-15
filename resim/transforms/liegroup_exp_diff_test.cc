#include "resim/transforms/liegroup_exp_diff.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "resim/transforms/so3.hh"

namespace resim::transforms {

TEST(ExpDiffTest, DerivativeOfExpSo3) {
  SO3::TangentVector all_zero = SO3::TangentVector::Zero();
  ExpDiffCoeffs coeffs_zero = derivative_of_exp_so3(all_zero.squaredNorm());
  // For zero case the coefficients are trivially known:
  constexpr double ONE = 1.;
  constexpr double HALF = 0.5;
  constexpr double SIXTH = 1. / 6;
  EXPECT_DOUBLE_EQ(coeffs_zero.a, ONE);
  EXPECT_DOUBLE_EQ(coeffs_zero.b, HALF);
  EXPECT_DOUBLE_EQ(coeffs_zero.c, SIXTH);

  // Testing the non-tiny angle code path for execution only. Correctness of
  // the output is covered by the general LieGroup tests and the SE3 tests.
  SO3::TangentVector all_ones = SO3::TangentVector::Ones();
  ExpDiffCoeffs coeffs_ones = derivative_of_exp_so3(all_ones.squaredNorm());
  constexpr double ZERO = 0;
  EXPECT_GT(coeffs_ones.a, ZERO);
  EXPECT_GT(coeffs_ones.b, ZERO);
  EXPECT_GT(coeffs_ones.c, ZERO);
}

}  // namespace resim::transforms
