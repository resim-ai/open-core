// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/dynamics/rigid_body/state.hh"

#include "resim/transforms/cross_matrix.hh"

namespace resim::dynamics::rigid_body {

using transforms::SE3;

State operator+(const State &state, const State::Delta &delta) {
  const auto from = state.reference_from_body.from();
  State result{state};
  result.reference_from_body =
      state.reference_from_body * SE3::exp(delta.head<SE3::DOF>(), from, from);
  result.d_reference_from_body += delta.tail<SE3::DOF>();
  return result;
}

State operator+(const State::Delta &delta, const State &state) {
  // Fall back to the above
  return state + delta;
}

State::Delta operator-(const State &state_a, const State &state_b) {
  State::Delta result;
  result.head<SE3::DOF>() =
      (state_b.reference_from_body.inverse() * state_a.reference_from_body)
          .log();
  result.tail<SE3::DOF>() =
      state_a.d_reference_from_body - state_b.d_reference_from_body;
  return result;
}

}  // namespace resim::dynamics::rigid_body
