// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <array>
#include <vector>

namespace resim::curves {

// Holds the polynomial coefficients for a TwoJet object.
// The coefficients should by applied by pre-multiplication in tangent
// space. This is trivial for the derivatives (e.g. a1 * d_frame_from_ref).
// In the case of the LieGroup the coefficient is applied as:
// Group::exp(a0 * frame_from_ref.log()); (as linear interpolation).
struct TwoJetPolyCoeffs {
  double a;    // for frame_from_ref()
  double da;   // for d_frame_from_ref()
  double d2a;  // for d2_frame_from_ref()
};

// Utility class to serve the quintic polynomial coefficients that are required
// for a quintic Hermite curve segment.
// Important assumptions:
// Curve segments are defined by their boundary conditions at a pair of TwoJet
// objects. These are called - respectively - the origin (orig) and the
// destination (dest). Time is normalized over the segment, so time_nrm = {0,
// 1}, for points between the origin (time_nrm = 0) and destination (time_nrm =
// 1).
class QuinticPolyCoeffs {
 public:
  // Constructing the quintic polynomial requires building and composing TwoJet
  // objects for each of the algebra elements representing the transform between
  // the origin and the destination and the first and second normalized-time
  // derivatives at both the origin and the destination. The methods below fetch
  // the TwoJetPolyCoeffs for each of these, given the value of normalized time
  // (time_nrm).
  static TwoJetPolyCoeffs dest_from_orig(double time_nrm);
  static TwoJetPolyCoeffs d_orig(double time_nrm);
  static TwoJetPolyCoeffs d_dest(double time_nrm);
  static TwoJetPolyCoeffs d2_orig(double time_nrm);
  static TwoJetPolyCoeffs d2_dest(double time_nrm);

 private:
  static constexpr unsigned int QUINTIC = 5;
  // Each coefficient 'a'  is calulated as
  // coeffs * {t, t^2, t^3, t^4, t^5}^T
  // If the coefficient is for a first of second time derivative (da, d2a),
  // then appropriate multiples (d_mult) are applied. See the comments against
  // the data members below for more detail. In the implementation, the
  // polynomial is evaluated using Horner's method.
  static double two_jet_single_coeff(
      const std::array<double, QUINTIC> &d_mult,
      const std::array<double, QUINTIC> &coeffs,
      double time_nrm,
      int smallest_index);

  // Compute coefficient a, da and d2a.
  static TwoJetPolyCoeffs two_jet_coeffs(
      const std::array<double, QUINTIC> &coeffs,
      double time_nrm);

  // Store the constants required to compute the coefficients.
  // A coefficient 'a' is calculated from and array 'c' below as:
  // a = c[0]t + c[1]t^2 ... +c[4]t^5
  static constexpr std::array<double, QUINTIC> DEST_ORIG{0., 0., 10., -15., 6.};
  static constexpr std::array<double, QUINTIC> D_ORIG{1., 0., -6., 8., -3.};
  static constexpr std::array<double, QUINTIC> D_DEST{0., 0., -4., 7., -3.};
  static constexpr std::array<double, QUINTIC> D2_ORIG{0., .5, -1.5, 1.5, -.5};
  static constexpr std::array<double, QUINTIC> D2_DEST{0., 0., .5, -1., .5};

  // To compute coefficients for the first and second time derivatives we need
  // to differentiate the expressions built using the coefficients above i.e:
  // a = c[0]t + c[1]t^2 ... + c[4]t^5 goes to:
  // da = c[0] + 2c[1]t ... 5c[4]t^4 and then to.
  // d2a = 2c[1] ... 20c[4]t^3.
  // The multiples being applied to the coefficients here are captured below.
  static constexpr std::array<double, QUINTIC> A{1, 1, 1, 1, 1};
  static constexpr std::array<double, QUINTIC> DA{1, 2, 3, 4, 5};
  static constexpr std::array<double, QUINTIC> D2A{0, 2, 6, 12, 20};
};

}  // namespace resim::curves
