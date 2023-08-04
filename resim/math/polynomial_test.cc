// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/math/polynomial.hh"

#include <gtest/gtest.h>

#include <cmath>
#include <sstream>

namespace resim::math {

namespace {

template <typename T>
class PolynomialTest : public ::testing::Test {};

template <std::size_t DEG>
struct Deg {
  static constexpr std::size_t Degree = DEG;
};

// NOLINTBEGIN(readability-magic-numbers)
using PolynomialDegrees = ::testing::Types<
    Deg<0U>,
    Deg<1U>,
    Deg<2U>,
    Deg<3U>,
    Deg<4U>,
    Deg<5U>,
    Deg<6U>,
    Deg<7U>,
    Deg<8U>,
    Deg<9U>,
    Deg<10U>>;
// NOLINTEND(readability-magic-numbers)

// A simple helper to compute the factorial
constexpr std::size_t factorial(const std::size_t n) {
  if (n > 0U) {
    return n * factorial(n - 1U);
  }
  return 1U;
}

// Make a Taylor series for exp(x)
template <std::size_t DEG, std::size_t... Idxs>
Polynomial<DEG> make_taylor_series(std::index_sequence<Idxs...> idxs) {
  return Polynomial<DEG>{{(1. / factorial(Idxs))...}};
};

}  // namespace

TYPED_TEST_SUITE(PolynomialTest, PolynomialDegrees);

TYPED_TEST(PolynomialTest, TestEvaluation) {
  // SETUP
  const Polynomial<TypeParam::Degree> polynomial{
      make_taylor_series<TypeParam::Degree>(
          std::make_index_sequence<TypeParam::Degree + 1U>{})};

  constexpr double EVAL_POINT = 0.5;

  // This is a different/simpler algorithm for evaluating the polynomial.
  double expected_value = 0.0;
  for (std::size_t ii = 0; ii < TypeParam::Degree + 1U; ++ii) {
    expected_value +=
        std::pow(EVAL_POINT, ii) / static_cast<double>(factorial(ii));
  }

  // ACTION / VERIFICATION
  EXPECT_DOUBLE_EQ(polynomial(EVAL_POINT), expected_value);
}

TYPED_TEST(PolynomialTest, Differentiation) {
  // SETUP
  const Polynomial<TypeParam::Degree> polynomial{
      make_taylor_series<TypeParam::Degree>(
          std::make_index_sequence<TypeParam::Degree + 1U>{})};

  // ACTION
  const auto d_polynomial{polynomial.prime()};

  // VERIFICATION
  // Verify that we just lost the highest order term
  constexpr double EVAL_POINT = 0.5;
  EXPECT_DOUBLE_EQ(
      d_polynomial(EVAL_POINT) + std::pow(EVAL_POINT, TypeParam::Degree) /
                                     factorial(TypeParam::Degree),
      polynomial(EVAL_POINT));
}

TYPED_TEST(PolynomialTest, Printing) {
  // SETUP
  std::ostringstream stream;

  constexpr std::size_t MAX_DEGREE = 10U;
  static constexpr std::array<const char *, MAX_DEGREE + 1U> EXPECTED_OUTPUT{
      "1",
      "1x + 1",
      "0.5x^2 + 1x + 1",
      "0.166667x^3 + 0.5x^2 + 1x + 1",
      "0.0416667x^4 + 0.166667x^3 + 0.5x^2 + 1x + 1",
      "0.00833333x^5 + 0.0416667x^4 + 0.166667x^3 + 0.5x^2 + 1x + 1",
      "0.00138889x^6 + 0.00833333x^5 + 0.0416667x^4 + 0.166667x^3 + 0.5x^2 + "
      "1x + 1",
      "0.000198413x^7 + 0.00138889x^6 + 0.00833333x^5 + 0.0416667x^4 + "
      "0.166667x^3 + 0.5x^2 + 1x + 1",
      "2.48016e-05x^8 + 0.000198413x^7 + 0.00138889x^6 + 0.00833333x^5 + "
      "0.0416667x^4 + 0.166667x^3 + 0.5x^2 + 1x + 1",
      "2.75573e-06x^9 + 2.48016e-05x^8 + 0.000198413x^7 + 0.00138889x^6 + "
      "0.00833333x^5 + 0.0416667x^4 + 0.166667x^3 + 0.5x^2 + 1x + 1",
      "2.75573e-07x^10 + 2.75573e-06x^9 + 2.48016e-05x^8 + 0.000198413x^7 + "
      "0.00138889x^6 + 0.00833333x^5 + 0.0416667x^4 + 0.166667x^3 + 0.5x^2 + "
      "1x + 1",
  };

  const Polynomial<TypeParam::Degree> polynomial{
      make_taylor_series<TypeParam::Degree>(
          std::make_index_sequence<TypeParam::Degree + 1U>{})};

  // ACTION
  stream << polynomial;

  // VERIFICATION
  EXPECT_EQ(stream.str(), EXPECTED_OUTPUT.at(TypeParam::Degree));
}

}  // namespace resim::math
