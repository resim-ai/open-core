//
// polynomial.hh
//
// This file defines a simple abstraction for a scalar polynomial with
// evaluation and differentiation.
//
#pragma once

#include <array>
#include <ostream>
#include <type_traits>

namespace resim::math {

// Forward declarations of Polynomial and its stream operator so we can define
// the stream operator as a friend despite it being a template.
template <std::size_t DEG>
class Polynomial;

template <std::size_t DEG>
std::ostream &operator<<(std::ostream &os, const Polynomial<DEG> &p);

template <std::size_t DEG>
class Polynomial {
 public:
  // Construct this polynomial from an array of coefficients.
  // @param[in] coeffs - The coefficients in order of ascending degree.
  explicit constexpr Polynomial(const std::array<double, DEG + 1U> &coeffs)
      : coeffs_{coeffs} {}

  // Evaluate this polynomial at x.
  double operator()(double x) const;

  // Differentiate this polynomial, returning its derivative as a
  // polynomial. For degree zero polynomials we return another degree zero
  // polynomial. For all other degrees, the degree is reduced by one.
  using DerivativeType = Polynomial<(DEG > 0U ? DEG - 1U : 0U)>;
  DerivativeType prime() const;

  friend std::ostream &operator<< <DEG>(
      std::ostream &os,
      const Polynomial<DEG> &p);

 private:
  std::array<double, DEG + 1U> coeffs_;
};

}  // namespace resim::math
