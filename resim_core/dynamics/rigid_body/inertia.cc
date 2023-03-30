#include "resim_core/dynamics/rigid_body/inertia.hh"

namespace resim::dynamics::rigid_body {

Inertia inertia_from_mass_and_moments_of_inertia(
    const double mass,
    const Eigen::Vector3d &moments_of_inertia) {
  using transforms::FSE3;

  const FSE3::TangentVector inertia_diagonal{FSE3::tangent_vector_from_parts(
      moments_of_inertia,
      mass * Eigen::Vector3d::Ones())};

  return inertia_diagonal.asDiagonal();
}

}  // namespace resim::dynamics::rigid_body
