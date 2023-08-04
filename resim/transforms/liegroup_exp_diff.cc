// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/transforms/liegroup_exp_diff.hh"

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

const TinyAngleConstCoeffs SE3_TINY_ANGLE_D{
    -1. / 12,
    -1. / 180,
    -1. / 6720,
};

const TinyAngleConstCoeffs SE3_TINY_ANGLE_E{
    -1. / 60,
    -1. / 1260,
    -1. / 60480,
};

}  // namespace

namespace resim::transforms {

enum class GroupTag {
  SO3,
  SE3,
};

ExpDiffCoeffs derivative_of_exp_impl(
    const double square_angle,
    const GroupTag tag) {
  ExpDiffCoeffs coeffs{};
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
    if (tag == GroupTag::SE3) {
      coeffs.d = tiny_exp_diff_coeff(SE3_TINY_ANGLE_D);
      coeffs.e = tiny_exp_diff_coeff(SE3_TINY_ANGLE_E);
    }
  } else {
    const double angle = std::sqrt(square_angle);
    const double inv_angle = 1. / angle;
    const double inv_square_angle = inv_angle * inv_angle;
    coeffs.a = std::sin(angle) * inv_angle;
    coeffs.b = (1. - std::cos(angle)) * inv_square_angle;
    coeffs.c = (1. - coeffs.a) * inv_square_angle;
    // NOLINTBEGIN(readability-magic-numbers)
    coeffs.d = (coeffs.a - 2. * coeffs.b) / square_angle;
    coeffs.e = (coeffs.b - 3. * coeffs.c) / square_angle;
    // NOLINTEND(readability-magic-numbers)
  }
  return coeffs;
}

ExpDiffCoeffs derivative_of_exp_so3(const double square_angle) {
  return derivative_of_exp_impl(square_angle, GroupTag::SO3);
}

ExpDiffCoeffs derivative_of_exp_se3(const double square_angle) {
  return derivative_of_exp_impl(square_angle, GroupTag::SE3);
}

}  // namespace resim::transforms
