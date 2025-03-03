// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <span>

#include "resim/curves/two_jet.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/status_value.hh"

namespace resim::curves::learning {

StatusValue<TwoJetL<resim::transforms::SE3>> two_jet_mean(
    const std::span<const TwoJetL<resim::transforms::SE3>> &samples,
    double tolerance,
    int max_iterations);

}  // namespace resim::curves::learning
