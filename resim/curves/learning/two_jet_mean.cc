// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/learning/two_jet_mean.hh"

#include "resim/assert/assert.hh"
#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/curves/two_jet.hh"

namespace resim::curves::learning {

using transforms::SE3;
using TwoJetL = TwoJetL<SE3>;
using TangentVector = optimization::TwoJetTangentVector<SE3>;

StatusValue<TwoJetL> two_jet_mean(
    const std::span<const TwoJetL> &samples,
    const double tolerance,
    const int max_iterations) {
  REASSERT(not samples.empty(), "Sample size must be greater than 1");
  REASSERT(
      max_iterations > 0,
      "Number of iterations must be greater than zero");

  TwoJetL guess = samples.front();
  TangentVector delta{TangentVector::Zero()};
  bool converged = false;
  for (int ii = 0; ii < max_iterations and not converged; ++ii) {
    for (const auto &sample : samples) {
      delta += optimization::difference(sample, guess);
    }
    delta /= static_cast<double>(samples.size());
    guess = optimization::accumulate(guess, delta);
    if (delta.norm() < tolerance) {
      converged = true;
    }
  }
  if (not converged) {
    return MAKE_STATUS("Failed to converge!");
  }
  return guess;
}

}  // namespace resim::curves::learning
