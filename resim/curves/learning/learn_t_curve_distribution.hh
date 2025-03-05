// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <functional>
#include <vector>

#include "resim/curves/learning/t_curve_distribution.hh"
#include "resim/curves/two_jet.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/status_value.hh"

namespace resim::curves::learning {

StatusValue<TCurveDistribution> learn_t_curve_distribution(
    const std::vector<double> &timestamps,
    const std::vector<
        std::function<StatusValue<TwoJetL<transforms::SE3>>(double)>> &curves);

}  // namespace resim::curves::learning
