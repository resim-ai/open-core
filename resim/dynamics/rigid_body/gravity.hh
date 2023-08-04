// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "au/au.hh"
#include "au/units/grams.hh"
#include "resim/dynamics/controller.hh"
#include "resim/dynamics/rigid_body/state.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/nullable_reference.hh"

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
