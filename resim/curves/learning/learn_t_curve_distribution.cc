// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/learning/learn_t_curve_distribution.hh"

#include <ranges>
#include <span>

#include "resim/assert/assert.hh"
#include "resim/curves/learning/t_curve_distribution.hh"
#include "resim/curves/learning/two_jet_mean.hh"
#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/curves/t_curve.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves::learning {

using transforms::SE3;

StatusValue<TCurveDistribution> learn_t_curve_distribution(
    const std::span<const double> &times,
    const std::span<
        const std::function<StatusValue<TwoJetL<transforms::SE3>>(double)>>
        &curves,
    const double mean_tolerance,
    const int mean_max_iterations) {
  using TwoJetL = TwoJetL<transforms::SE3>;
  const std::size_t num_times = times.size();
  const std::size_t num_curves = curves.size();

  REASSERT(num_times > 0, "There must be at least 1 time");
  REASSERT(num_curves > 1, "There must be at least 2 curves");
  // Sample the functions
  std::vector<TwoJetL> samples;
  samples.reserve(num_times * num_curves);
  for (std::size_t ii = 0; ii < num_times; ++ii) {
    const double time = times[ii];
    for (std::size_t jj = 0; jj < num_curves; ++jj) {
      const auto &curve = curves[jj];
      samples.push_back(RETURN_OR_ASSIGN(curve(time)));
    }
  }

  // Compute the means:
  std::vector<TCurve<SE3>::Control> means;
  means.reserve(num_times);
  for (std::size_t ii = 0; ii < num_times; ++ii) {
    const std::span<const TwoJetL> poses_at_time{
        std::next(samples.cbegin(), static_cast<int>(ii * num_curves)),
        num_curves};

    means.emplace_back(TCurve<SE3>::Control{
        .time = times[ii],
        .point = RETURN_OR_ASSIGN(
            two_jet_mean(poses_at_time, mean_tolerance, mean_max_iterations))});
  }

  // Compute the covariance
  constexpr int DOF = optimization::TWO_JET_DOF<SE3>;
  Eigen::MatrixXd mean_deviation = Eigen::MatrixXd::Zero(
      static_cast<int>(DOF * num_times),
      static_cast<int>(num_curves));

  for (int ii = 0; ii < num_times; ++ii) {
    const auto &mean = means.at(ii).point;
    for (int jj = 0; jj < num_curves; ++jj) {
      const int offset = ii * DOF;
      mean_deviation.block<DOF, 1>(offset, jj) =
          optimization::difference(samples.at(ii * num_curves + jj), mean);
    }
  }
  return TCurveDistribution{
      .mean = TCurve<SE3>{means},
      .covariance =
          mean_deviation * mean_deviation.transpose() / (num_curves - 1),
  };
}

}  // namespace resim::curves::learning
