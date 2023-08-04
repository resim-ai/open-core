// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/dynamics/rigid_body/dynamics.hh"

#include <utility>

#include "resim/assert/assert.hh"

namespace resim::dynamics::rigid_body {

Dynamics::Dynamics(Inertia inertia)
    : inertia_{std::move(inertia)},
      inertia_inv_{inertia_.inverse()} {}

Dynamics::Delta Dynamics::operator()(
    const State &state,
    const Control &control,
    time::Timestamp time,
    NullableReference<Diffs> diffs) const {
  using transforms::SE3;
  using TangentVector = SE3::TangentVector;

  // TODO(michael) Add the Jacobian to this
  REASSERT(not diffs.has_value(), "Jacobian not (yet) supported!");

  const TangentVector &force = control;
  const TangentVector acceleration = [&I = inertia_,
                                      &Iinv = inertia_inv_,
                                      &V = state.d_reference_from_body,
                                      &F = force]() -> TangentVector {
    // We're using equation 31 from
    // https://drive.google.com/file/d/1CBpHs88MsTx1971AWM3SzPOsAeba1VD1/view
    // I believe this is a performance hotspot based on running the
    // unit test, but I haven't optimized it for now since it may not
    // be a bottlneck in real sims. To make this faster, one can
    // compute the adjoint term using cross products without having to
    // form a 6x6 matrix. That almost 2x's performance in the unit test.
    return Iinv * (F + SE3::adjoint(V).transpose() * I * V);
  }();
  return (Delta() << state.d_reference_from_body, acceleration).finished();
}

}  // namespace resim::dynamics::rigid_body
