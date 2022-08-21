#include "curves/quintic_poly_coeffs.hh"

namespace resim::curves {

TwoJetPolyCoeffs QuinticPolyCoeffs::dest_from_orig(const double n_time) {
  return two_jet_coeffs(DEST_ORIG, n_time);
}
TwoJetPolyCoeffs QuinticPolyCoeffs::d_orig(const double n_time) {
  return two_jet_coeffs(D_ORIG, n_time);
}
TwoJetPolyCoeffs QuinticPolyCoeffs::d_dest(const double n_time) {
  return two_jet_coeffs(D_DEST, n_time);
}
TwoJetPolyCoeffs QuinticPolyCoeffs::d2_orig(const double n_time) {
  return two_jet_coeffs(D2_ORIG, n_time);
}
TwoJetPolyCoeffs QuinticPolyCoeffs::d2_dest(const double n_time) {
  return two_jet_coeffs(D2_DEST, n_time);
}

double QuinticPolyCoeffs::two_jet_single_coeff(
    const std::array<double, QUINTIC> &d_mult,
    const std::array<double, QUINTIC> &coeffs,
    const double n_time,
    int smallest_index) {
  double coeff = 0.;
  for (int i = QUINTIC - 1; i >= smallest_index; --i) {
    coeff = n_time * (d_mult.at(i) * coeffs.at(i) + coeff);
  }
  return coeff;
}

TwoJetPolyCoeffs QuinticPolyCoeffs::two_jet_coeffs(
    const std::array<double, QUINTIC> &coeffs,
    const double n_time) {
  const double a0 = two_jet_single_coeff(A, coeffs, n_time, 0);
  const double a1 =
      DA.at(0) * coeffs.at(0) + two_jet_single_coeff(DA, coeffs, n_time, 1);
  const double a2 =
      D2A.at(1) * coeffs.at(1) + two_jet_single_coeff(D2A, coeffs, n_time, 2);
  return {a0, a1, a2};
}

}  // namespace resim::curves
