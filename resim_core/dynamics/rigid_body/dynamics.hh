#pragma once

#include <Eigen/Dense>

#include "resim_core/dynamics/dynamics.hh"
#include "resim_core/dynamics/rigid_body/inertia.hh"
#include "resim_core/dynamics/rigid_body/state.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::dynamics::rigid_body {

//
// This class defines the dynamics for a rigid body using the generalized force
// (6-DoF combined torque and linear force) in body coordinates as the control
// vector.
//
class Dynamics : public dynamics::Dynamics<State, transforms::SE3::DOF> {
 public:
  // Constructor.
  // @param[in] inertia - The 6x6 inertia matrix for this rigid body
  explicit Dynamics(Inertia inertia);

  // The dynamics themselves.
  // These dynamics compute the acceleration that ensures that (d/dt)[inertia_ *
  // state.d_reference_from_body] is equal to the six degree of freedom force
  // passed in as the control vector, *assuming that inertia is constant*.
  // @param[in] state - The current rigid body state.
  // @param[in] control - The 6-DoF generalized force on this rigid body.
  // @param[in] time - The current time.
  // @param[out] diffs - The optional Jacobians of this function w.r.t. state
  //                     and control
  Delta operator()(
      const State &state,
      const Control &control,
      time::Timestamp time,
      NullableReference<Diffs> diffs) const override;

 private:
  Inertia inertia_{Inertia::Identity()};
  // The inverse of the inertia. Cached for performance.
  Inertia inertia_inv_{Inertia::Identity()};
};

}  // namespace resim::dynamics::rigid_body
