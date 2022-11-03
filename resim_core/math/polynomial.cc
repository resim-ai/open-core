#include "resim_core/math/polynomial.hh"

#include <cmath>

namespace resim::math {

template <std::size_t DEG>
double Polynomial<DEG>::operator()(const double x) const {
  // Horner's Method
  double result = coeffs_[DEG];
  for (std::size_t ii = DEG - 1U; ii + 1U > 0U; --ii) {
    result = (x * result) + coeffs_.at(ii);
  }
  return result;
}

// The above function has a branch which never executes for DEG == 0
// so we specialize it for DEG == 0 to ensure we don't have any
// uncovered branches.
template <>
double Polynomial<0>::operator()(const double x) const {
  return coeffs_[0];
}

template <std::size_t DEG>
typename Polynomial<DEG>::DerivativeType Polynomial<DEG>::prime() const {
  std::array<double, DEG> result_coeffs{};

  for (std::size_t ii = 0; ii < DEG; ++ii) {
    result_coeffs.at(ii) = (ii + 1U) * coeffs_.at(ii + 1U);
  }
  return Polynomial<DEG - 1U>{result_coeffs};
}

template <>
Polynomial<0> Polynomial<0>::prime() const {
  // Derivative of a constant is zero.
  return Polynomial<0>{{0.}};
}

template <std::size_t DEG>
std::ostream &operator<<(std::ostream &os, const Polynomial<DEG> &p) {
  for (std::size_t ii = DEG; ii + 1U > 0U; --ii) {
    os << p.coeffs_.at(ii);
    if (ii == 1U) {
      os << "x + ";
    }
    if (ii > 1U) {
      os << "x^" << ii << " + ";
    }
  }
  return os;
}

// The above definition has branches that never execute for DEG == 0
// and DEG == 1, so we add these specializations to ensure we dont'
// have branches not covered by tests.
template <>
std::ostream &operator<<(std::ostream &os, const Polynomial<0> &p) {
  return os << p.coeffs_[0];
}
template <>
std::ostream &operator<<(std::ostream &os, const Polynomial<1> &p) {
  return os << p.coeffs_[1] << "x + " << p.coeffs_[0];
}

template std::ostream &operator<<(std::ostream &os, const Polynomial<2> &p);
template std::ostream &operator<<(std::ostream &os, const Polynomial<3> &p);
template std::ostream &operator<<(std::ostream &os, const Polynomial<4> &p);
template std::ostream &operator<<(std::ostream &os, const Polynomial<5> &p);
template std::ostream &operator<<(std::ostream &os, const Polynomial<6> &p);
template std::ostream &operator<<(std::ostream &os, const Polynomial<7> &p);
template std::ostream &operator<<(std::ostream &os, const Polynomial<8> &p);
template std::ostream &operator<<(std::ostream &os, const Polynomial<9> &p);
template std::ostream &operator<<(std::ostream &os, const Polynomial<10> &p);

// NOLINTBEGIN(readability-magic-numbers)
template class Polynomial<0>;
template class Polynomial<1>;
template class Polynomial<2>;
template class Polynomial<3>;
template class Polynomial<4>;
template class Polynomial<5>;
template class Polynomial<6>;
template class Polynomial<7>;
template class Polynomial<8>;
template class Polynomial<9>;
template class Polynomial<10>;
// NOLINTEND(readability-magic-numbers)

}  // namespace resim::math
