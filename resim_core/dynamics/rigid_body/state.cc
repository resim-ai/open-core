
#include "resim_core/dynamics/rigid_body/state.hh"

#include <iostream>

#include "resim_core/transforms/cross_matrix.hh"

namespace resim::dynamics::rigid_body {

using transforms::FSE3;

State operator+(const State &state, const State::Delta &delta) {
  const auto from = state.reference_from_body.from();
  State result{state};
  result.reference_from_body =
      state.reference_from_body *
      FSE3{transforms::SE3::exp(delta.head<FSE3::DOF>()), from, from};
  result.d_reference_from_body += delta.tail<FSE3::DOF>();
  return result;
}

State operator+(const State::Delta &delta, const State &state) {
  // Fall back to the above
  return state + delta;
}

State::Delta operator-(const State &state_a, const State &state_b) {
  State::Delta result;
  result.head<FSE3::DOF>() =
      (state_b.reference_from_body.inverse() * state_a.reference_from_body)
          .log();
  result.tail<FSE3::DOF>() =
      state_a.d_reference_from_body - state_b.d_reference_from_body;
  return result;
}

}  // namespace resim::dynamics::rigid_body
