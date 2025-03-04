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

// Compute the mean of a set of two jets. Since two jets exist on a manifold,
// this function computes the two jet u such that sum_i(x_i (-) u) = 0 where (-)
// is defined as the left minus operator as defined in
// resim/curves/optimization/two_jet_tangent_space.hh. Another way of saying
// this is that u is the mean of all the samples when they're expressed in u's
// tangent space. We use iteration to find u  and therefore a tolerance and a
// maximum number of iterations must be provided.
// @param[in] samples - The samples to compute the mean of.
// @param[in] tolerance - The absolute tolerance for computing the mean.
//                        Iteration terminates once updates get smaller in L2
//                        norm than this.
// @param[in] max_iterations - The maximum number of iterations allowed before
//                             terminating unsuccessfully.
// @returns The mean as described above, or a bad status otherwise.
StatusValue<TwoJetL<resim::transforms::SE3>> two_jet_mean(
    const std::span<const TwoJetL<resim::transforms::SE3>> &samples,
    double tolerance,
    int max_iterations);

}  // namespace resim::curves::learning
