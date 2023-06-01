#pragma once

#include "au/au.hh"
#include "au/units/grams.hh"
#include "resim_core/dynamics/controller.hh"
#include "resim_core/dynamics/rigid_body/state.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/nullable_reference.hh"

namespace resim::dynamics::rigid_body {

//
// Add a force function which computes gravity for a rigid body, assuming that
// gravity pulls downwards along the Z axis.
//
class GravityForce : public Controller<State, transforms::SE3::DOF> {
 public:
  // Construct using the mass.
  explicit GravityForce(au::QuantityD<au::Kilo<au::Grams>> mass);

  // Compute the gravity based on the current state.
  Control operator()(
      const State &state,
      time::Timestamp time,
      NullableReference<Jacobian> jacobian) const override;

 private:
  au::QuantityD<au::Kilo<au::Grams>> mass_;
};

}  // namespace resim::dynamics::rigid_body
