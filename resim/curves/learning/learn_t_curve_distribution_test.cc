// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/learning/learn_t_curve_distribution.hh"

#include <gtest/gtest.h>

#include "resim/time/sample_interval.hh"

namespace resim::curves::learning {

using transforms::SE3;
using TwoJetL = TwoJetL<SE3>;

TEST(LearnTCurveDistributionTest, TestLearnTCurveDistribution) {
  // SETUP
  constexpr double START_TIME = 0.0;
  constexpr double END_TIME = 10.0;
  constexpr double MAX_ABS_DT = 1.5;
  std::vector<double> times;
  time::sample_interval(
      START_TIME,
      END_TIME,
      MAX_ABS_DT,
      [&times](const double time) { times.push_back(time); });

  std::vector<std::function<StatusValue<TwoJetL>(double)>> curves;
  curves.emplace_back([](const double t) -> StatusValue<TwoJetL> {
    std::cout << "ASDF" << std::endl;
    return TwoJetL::identity();
  });

  // ACTION

  auto maybe_distribution = learn_t_curve_distribution(
      std::views::all(times),
      std::views::all(curves));
}

}  // namespace resim::curves::learning
