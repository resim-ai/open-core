#include "resim_core/curves/quintic_poly_coeffs.hh"

#include <gtest/gtest.h>

namespace resim::curves {

namespace {

struct HandComputedPolyCoeffs {
  TwoJetPolyCoeffs dest_from_orig;
  TwoJetPolyCoeffs d_orig;
  TwoJetPolyCoeffs d_dest;
  TwoJetPolyCoeffs d2_orig;
  TwoJetPolyCoeffs d2_dest;
};

constexpr double ZERO = 0.;
constexpr double HALF = 0.5;
constexpr double ONE = 1.;

const HandComputedPolyCoeffs COEFFSTZERO{
    {0., 0., 0.},
    {0., 1., 0.},
    {0., 0., 0.},
    {0., 0., 1.},
    {0., 0., 0.}};

const HandComputedPolyCoeffs COEFFSTONE{
    {1., 0., 0.},
    {0., 0., 0.},
    {0., 1., 0.},
    {0., 0., 0.},
    {0., 0., 1.}};

const HandComputedPolyCoeffs COEFFSTHALF{
    {1 / 2., 15 / 8., 0.},
    {5 / 32., -7 / 16., -3 / 2.},
    {-5 / 32., -7 / 16., 3 / 2.},
    {1 / 64., -1 / 32., -1 / 4.},
    {1 / 64., 1 / 32., -1 / 4.}};

}  // namespace

void helper_two_jet_poly_coeffs_eq(
    const TwoJetPolyCoeffs &left,
    const TwoJetPolyCoeffs &right) {
  EXPECT_DOUBLE_EQ(left.a, right.a);
  EXPECT_DOUBLE_EQ(left.da, right.da);
  EXPECT_DOUBLE_EQ(left.d2a, right.d2a);
}

void helper_compare_coeffs(
    const HandComputedPolyCoeffs &ref_coeffs,
    double time) {
  const auto dest_from_orig = QuinticPolyCoeffs::dest_from_orig(time);
  helper_two_jet_poly_coeffs_eq(ref_coeffs.dest_from_orig, dest_from_orig);

  const auto d_orig = QuinticPolyCoeffs::d_orig(time);
  helper_two_jet_poly_coeffs_eq(ref_coeffs.d_orig, d_orig);

  const auto d_dest = QuinticPolyCoeffs::d_dest(time);
  helper_two_jet_poly_coeffs_eq(ref_coeffs.d_dest, d_dest);

  const auto d2_orig = QuinticPolyCoeffs::d2_orig(time);
  helper_two_jet_poly_coeffs_eq(ref_coeffs.d2_orig, d2_orig);

  const auto d2_dest = QuinticPolyCoeffs::d2_dest(time);
  helper_two_jet_poly_coeffs_eq(ref_coeffs.d2_dest, d2_dest);
}

TEST(QuintiPolyTest, TestTZero) { helper_compare_coeffs(COEFFSTZERO, ZERO); }

TEST(QuintiPolyTest, TestTOne) { helper_compare_coeffs(COEFFSTONE, ONE); }

TEST(QuintiPolyTest, TestTHalf) { helper_compare_coeffs(COEFFSTHALF, HALF); }

}  // namespace resim::curves
