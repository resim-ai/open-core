#include "transforms/liegroup_exp_diff.hh"

#include <cmath>

namespace {

struct TinyAngleConstCoeffs {
  double a;
  double b;
  double c;
};

const TinyAngleConstCoeffs SO3_TINY_ANGLE_B{
    0.5,
    1. / 24,
    1. / 720,
};

const TinyAngleConstCoeffs SO3_TINY_ANGLE_C{
    1. / 6,
    1. / 120,
    1. / 5040,
};

}  // namespace

namespace resim {
namespace transforms {

ExpDiffCoeffs derivative_of_exp_so3(const double square_angle) {
  ExpDiffCoeffs coeffs;
  if (square_angle < TINY_SQUARE_ANGLE) {
    const auto tiny_exp_diff_coeff =
        [&square_angle](const TinyAngleConstCoeffs &tiny_coeffs) -> double {
      return (
          tiny_coeffs.a -
          square_angle * (tiny_coeffs.b - square_angle * tiny_coeffs.c));
    };
    coeffs.b = tiny_exp_diff_coeff(SO3_TINY_ANGLE_B);
    coeffs.c = tiny_exp_diff_coeff(SO3_TINY_ANGLE_C);
    coeffs.a = 1. - square_angle * coeffs.c;
  } else {
    const double angle = std::sqrt(square_angle);
    const double inv_angle = 1. / angle;
    const double inv_square_angle = inv_angle * inv_angle;
    coeffs.a = std::sin(angle) * inv_angle;
    coeffs.b = (1. - std::cos(angle)) * inv_square_angle;
    coeffs.c = (1. - coeffs.a) * inv_square_angle;
  }
  return coeffs;
}

}  // namespace transforms
}  // namespace resim
