// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <random>

#include "resim/converter/fuzz_helpers.hh"
#include "resim/converter/tags.hh"
#include "resim/curves/learning/t_curve_distribution.hh"
#include "resim/curves/t_curve.hh"
#include "resim/curves/two_jet.hh"
#include "resim/math/is_approx.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/inout.hh"

namespace resim::curves::learning {

template <typename Rng>
TCurveDistribution random_element(
    converter::TypeTag<TCurveDistribution> /*unused*/,
    InOut<Rng> rng) {
  constexpr std::size_t LB = 2U;
  constexpr std::size_t UB = 10U;
  std::uniform_int_distribution<std::size_t> dist{LB, UB};
  const std::size_t num_control_points = dist(*rng);

  std::vector<TCurve<transforms::SE3>::Control> control_points;
  control_points.reserve(num_control_points);
  for (std::size_t ii = 0U; ii < num_control_points; ++ii) {
    control_points.emplace_back(TCurve<transforms::SE3>::Control{
        .time = static_cast<double>(ii),
        .point =
            TwoJetL{
                transforms::SE3::exp(
                    random_element<transforms::SE3::TangentVector>(rng)),
                random_element<transforms::SE3::TangentVector>(rng),
                random_element<transforms::SE3::TangentVector>(rng)},
    });
  }

  return TCurveDistribution{
      .mean = TCurve<transforms::SE3>{control_points},
  };
}

bool custom_verify_equality(
    const TCurveDistribution &a,
    const TCurveDistribution &b);

}  // namespace resim::curves::learning
