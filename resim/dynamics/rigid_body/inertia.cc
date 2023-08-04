// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/dynamics/rigid_body/inertia.hh"

namespace resim::dynamics::rigid_body {

Inertia inertia_from_mass_and_moments_of_inertia(
    const double mass,
    const Eigen::Vector3d &moments_of_inertia) {
  using transforms::SE3;

  const SE3::TangentVector inertia_diagonal{SE3::tangent_vector_from_parts(
      moments_of_inertia,
      mass * Eigen::Vector3d::Ones())};

  return inertia_diagonal.asDiagonal();
}

}  // namespace resim::dynamics::rigid_body
