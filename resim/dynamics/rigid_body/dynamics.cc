// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/dynamics/rigid_body/dynamics.hh"

#include <utility>

#include "resim/assert/assert.hh"

namespace resim::dynamics::rigid_body {

using transforms::SE3;
using TangentVector = SE3::TangentVector;

namespace {

// Helper function which returns the result of the tensor contraction (d ad_V /
// dV) * x.
Eigen::Matrix<double, SE3::DOF, SE3::DOF> d_adVT_dV(const TangentVector &x) {
  Eigen::Matrix<double, SE3::DOF, SE3::DOF> result{
      Eigen::Matrix<double, SE3::DOF, SE3::DOF>::Zero()};
  struct TensorEntry {
    int i, j, k;
    double val;
  };

  constexpr std::array d_cross = {
      TensorEntry{.i = 1, .j = 2, .k = 0, .val = 1.0},
      TensorEntry{.i = 2, .j = 1, .k = 0, .val = -1.0},
      TensorEntry{.i = 0, .j = 2, .k = 1, .val = -1.0},
      TensorEntry{.i = 2, .j = 0, .k = 1, .val = 1.0},
      TensorEntry{.i = 0, .j = 1, .k = 2, .val = 1.0},
      TensorEntry{.i = 1, .j = 0, .k = 2, .val = -1.0},
  };

  for (const auto &entry : d_cross) {
    // Upper left submatrix
    result(entry.i, entry.k) += entry.val * x(entry.j);

    // Upper right submatrix
    result(entry.i, entry.k + 3) += entry.val * x(entry.j + 3);

    // Lower right submatrix
    result(entry.i + 3, entry.k) += entry.val * x(entry.j + 3);
  }

  return result;
}

}  // namespace

Dynamics::Dynamics(Inertia inertia)
    : inertia_{std::move(inertia)},
      inertia_inv_{inertia_.inverse()} {}

Dynamics::Delta Dynamics::operator()(
    const State &state,
    const Control &control,
    time::Timestamp time,
    NullableReference<Diffs> diffs) const {
  // TODO(michael) Add the Jacobian to this

  if (diffs.has_value()) {
    // TODO might be a more efficient way to do this
    [&f_x = diffs->f_x,
     &f_u = diffs->f_u,
     &I = inertia_,
     &Iinv = inertia_inv_,
     &V = state.d_reference_from_body]() {
      f_x = MatXX::Zero();
      f_x.block<SE3::DOF, SE3::DOF>(SE3::DOF, SE3::DOF) =
          Iinv * (SE3::adjoint(V).transpose() * I + d_adVT_dV(I * V));
      f_x.block<SE3::DOF, SE3::DOF>(0, SE3::DOF) =
          SE3::TangentMapping::Identity();

      f_u = MatXU::Zero();
      f_u.block<SE3::DOF, SE3::DOF>(SE3::DOF, 0) = Iinv;
    }();
  }

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
