// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <functional>
#include <span>

#include "resim/curves/learning/t_curve_distribution.hh"
#include "resim/curves/two_jet.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/status_value.hh"

namespace resim::curves::learning {

// This function fits a distribution to a sequence of curves expressed as
// functions from time to TwoJetL's. These functions can also return bad
// statuses in which case the fitting fails. The result is a TCurveDistribution
// with control points at the specified times. The times are assumed to be valid
// for all of the provided curves.
// @param[in] times - The times to sample at for the fit.
// @parma[in] curves - The curves we're trying to learn from.
// @throws AssertException if finding the mean of the curves doesn't converge,
//         if there are fewer than two curves, if there isn't at least one time,
//         or if any of the curves fail when queried.
// @returns A TCurve fit to the given trajectories with the given times.
StatusValue<TCurveDistribution> learn_t_curve_distribution(
    const std::span<const double> &times,
    const std::span<
        const std::function<StatusValue<TwoJetL<transforms::SE3>>(double)>>
        &curves,
    double mean_tolerance,
    int mean_max_iterations);

}  // namespace resim::curves::learning
