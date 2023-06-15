#include "resim/curves/quintic_poly_coeffs.hh"

namespace resim::curves {

TwoJetPolyCoeffs QuinticPolyCoeffs::dest_from_orig(const double time_nrm) {
  return two_jet_coeffs(DEST_ORIG, time_nrm);
}
TwoJetPolyCoeffs QuinticPolyCoeffs::d_orig(const double time_nrm) {
  return two_jet_coeffs(D_ORIG, time_nrm);
}
TwoJetPolyCoeffs QuinticPolyCoeffs::d_dest(const double time_nrm) {
  return two_jet_coeffs(D_DEST, time_nrm);
}
TwoJetPolyCoeffs QuinticPolyCoeffs::d2_orig(const double time_nrm) {
  return two_jet_coeffs(D2_ORIG, time_nrm);
}
TwoJetPolyCoeffs QuinticPolyCoeffs::d2_dest(const double time_nrm) {
  return two_jet_coeffs(D2_DEST, time_nrm);
}

double QuinticPolyCoeffs::two_jet_single_coeff(
    const std::array<double, QUINTIC> &d_mult,
    const std::array<double, QUINTIC> &coeffs,
    const double time_nrm,
    int smallest_index) {
  double coeff = 0.;
  for (int i = QUINTIC - 1; i >= smallest_index; --i) {
    coeff = time_nrm * (d_mult.at(i) * coeffs.at(i) + coeff);
  }
  return coeff;
}

TwoJetPolyCoeffs QuinticPolyCoeffs::two_jet_coeffs(
    const std::array<double, QUINTIC> &coeffs,
    const double time_nrm) {
  const double a0 = two_jet_single_coeff(A, coeffs, time_nrm, 0);
  const double a1 =
      DA.at(0) * coeffs.at(0) + two_jet_single_coeff(DA, coeffs, time_nrm, 1);
  const double a2 =
      D2A.at(1) * coeffs.at(1) + two_jet_single_coeff(D2A, coeffs, time_nrm, 2);
  return {a0, a1, a2};
}

}  // namespace resim::curves
