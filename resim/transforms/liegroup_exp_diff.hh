#pragma once

#include <optional>
namespace resim::transforms {
// Define a very small angle threshold for logic avoiding div 0 issues.
constexpr double TINY_SQUARE_ANGLE = 4e-6;

// This library contains some helper functions to compute the derivatives
// of the exp and log mapping functions for LieGroups (SO3 and SE3).
// The naming conventions used follow the document "Derivative of the
// Exponential Map", available here: https://ethaneade.com/exp_diff.pdf

// Computing the derivatives of exp for both SO3 and SE3 involves computing
// an expression of the form a*I + b*omega + c*omega*omegaT, where a, b c are
// coefficients. See eq. 53 and eq. 70 in https://ethaneade.com/exp_diff.pdf.
// This struct is a useful container for holding these coefficients.
struct ExpDiffCoeffs {
  double a = 0.;
  double b = 0.;
  double c = 0.;
  // Optional coefficients used for SE3
  std::optional<double> d;  // (a - 2b)/angle^2
  std::optional<double> e;  // (b - 3c)/angle^2
};

// Returns the exponential map coefficients for computing the derivative
// of exp for SO3. See eq. 53 in https://ethaneade.com/exp_diff.pdf.
// [param] square_angle - The square of the angle of rotation. Also the squared
//                        norm of the algebra element.
ExpDiffCoeffs derivative_of_exp_so3(double square_angle);

// Returns the exponential map coefficients for computing the derivative
// of exp for SE3. See eq. 71 in https://ethaneade.com/exp_diff.pdf.
// [param] square_angle - The square of the angle of rotation. Also the squared
//                        norm of the rotational part of the algebra element.
ExpDiffCoeffs derivative_of_exp_se3(double square_angle);
}  // namespace resim::transforms
