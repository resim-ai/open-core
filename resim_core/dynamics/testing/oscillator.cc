#include "resim_core/dynamics/testing/oscillator.hh"

namespace resim::dynamics::testing {

////////////////////////////////////////////////////////////////////////////////
// Free Functions                                                             //
////////////////////////////////////////////////////////////////////////////////

OscillatorState operator+(
    const OscillatorState &state,
    const Eigen::Vector2d &delta) {
  return OscillatorState{
      .displacement = state.displacement + delta.x(),
      .velocity = state.velocity + delta.y(),
  };
}

////////////////////////////////////////////////////////////////////////////////
// Oscillator Dynamics Implementation                                         //
////////////////////////////////////////////////////////////////////////////////

OscillatorDynamics::Delta OscillatorDynamics::operator()(
    const State &state,
    const Control &control,
    const time::Timestamp time,
    NullableReference<Diffs> jacobian) const {
  if (jacobian.has_value()) {
    // f_x should be:
    //
    // [0., 1.]
    // [0., 0.]
    //
    // This is because only the position derivative depends directly on the
    // state, and it only depends on the velocity. There is an indirect
    // dependence of the velocity derivative on the displacement through the
    // controller, but that's handled through the control jacobian f_u and the
    // chain rule.
    jacobian->f_x << 0., 1., 0., 0.;

    // f_u should be:
    //
    // [0.]
    // [1.]
    //
    jacobian->f_u << 0., 1.;
  }
  return Delta{state.velocity, control.x()};
}

////////////////////////////////////////////////////////////////////////////////
// Oscillator Controller Implementation                                       //
////////////////////////////////////////////////////////////////////////////////

OscillatorController::OscillatorController(const double stiffness)
    : stiffness_{stiffness} {}

OscillatorController::Control OscillatorController::operator()(
    const OscillatorState &state,
    const time::Timestamp time,
    const NullableReference<Jacobian> jacobian) const {
  if (jacobian.has_value()) {
    // The control Jacobian is just:
    // [-k, 0.]
    *jacobian << -stiffness_, 0.;
  }
  return Control{-stiffness_ * state.displacement};
}

}  // namespace resim::dynamics::testing
