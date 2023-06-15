
#include "resim/dynamics/rigid_body/gravity.hh"

#include <Eigen/Dense>

#include "au/units/newtons.hh"
#include "resim/assert/assert.hh"
#include "resim/dynamics/constants.hh"
#include "resim/transforms/framed_vector.hh"
#include "resim/transforms/se3.hh"

namespace resim::dynamics::rigid_body {

GravityForce::GravityForce(const au::QuantityD<au::Kilo<au::Grams>> mass)
    : mass_{mass} {}

GravityForce::Control GravityForce::operator()(
    const State &state,
    time::Timestamp time,
    NullableReference<Jacobian> jacobian) const {
  REASSERT(not jacobian.has_value(), "Jacobian not yet supported!");
  using Vec3 = Eigen::Vector3d;

  // Newton's Second Law
  const auto force = mass_ * GRAVITY_ACCELERATION;
  const transforms::FramedVector<transforms::SE3::DIMS> gravity_in_scene_frame{
      -force.in(au::newtons)*Vec3::UnitZ(),
      state.reference_from_body.into()};

  const transforms::FramedVector<transforms::SE3::DIMS> gravity_in_body_frame{
      state.reference_from_body.rotation().inverse() * gravity_in_scene_frame,
      state.reference_from_body.from()};

  return transforms::SE3::tangent_vector_from_parts(
      Vec3::Zero(),
      gravity_in_body_frame);
}

}  // namespace resim::dynamics::rigid_body
