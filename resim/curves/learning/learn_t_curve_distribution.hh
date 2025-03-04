// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <functional>
#include <ranges>

#include "resim/curves/learning/t_curve_distribution.hh"
#include "resim/curves/two_jet.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/status_value.hh"

namespace resim::curves::learning {

template <std::ranges::view TimeView, std::ranges::view CurveView>
requires std::same_as < std::ranges::range_value_t<TimeView>,
double > &&std::same_as<
             std::ranges::range_value_t<CurveView>,
             std::function<StatusValue<TwoJetL<transforms::SE3>>(double)>>
             StatusValue<TCurveDistribution> learn_t_curve_distribution(
                 const TimeView &timestamps,
                 const CurveView &curves) {
  for (const auto &time : timestamps) {
    for (const auto &curve : curves) {
      RETURN_OR_ASSIGN(curve(time));
    }
  }
  return MAKE_STATUS("Not implemented!");
}

}  // namespace resim::curves::learning
