// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/learning/learn_t_curve_distribution.hh"

#include <iostream>  // TODO
#include <ranges>
#include <span>

#include "resim/curves/learning/t_curve_distribution.hh"
#include "resim/curves/learning/two_jet_mean.hh"

namespace resim::curves::learning {

StatusValue<TCurveDistribution> learn_t_curve_distribution(
    const std::vector<double> &timestamps,
    const std::vector<
        std::function<StatusValue<TwoJetL<transforms::SE3>>(double)>> &curves) {
  using TwoJetL = TwoJetL<transforms::SE3>;
  const std::size_t num_times = timestamps.size();
  const std::size_t num_curves = curves.size();

  std::vector<TwoJetL> samples;
  samples.reserve(num_times * num_curves);
  for (std::size_t ii = 0; ii < num_times; ++ii) {
    const double time = timestamps.at(ii);
    for (std::size_t jj = 0; jj < num_curves; ++jj) {
      const auto &curve = curves.at(jj);
      samples.push_back(RETURN_OR_ASSIGN(curve(time)));
    }
  }

  constexpr double TOLERANCE = 1e-4;
  constexpr int MAX_ITERATIONS = 15;
  std::vector<TwoJetL> means;
  means.reserve(num_times);
  for (std::size_t ii = 0; ii < num_times; ++ii) {
    const std::span<const TwoJetL> poses_at_time{
        std::views::counted(samples.cbegin() + ii * num_curves, num_curves)};

    means.emplace_back(RETURN_OR_ASSIGN(
        two_jet_mean(poses_at_time, TOLERANCE, MAX_ITERATIONS)));
  }

  for (const auto &m : means) {
    std::cout << m.frame_from_ref().log().transpose() << std::endl;
  }

  return TCurveDistribution();
}

}  // namespace resim::curves::learning
