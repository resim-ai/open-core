#include "resim/dynamics/aerodynamics/rigid_body_aerodynamics.hh"

#include <Eigen/Dense>
#include <functional>
#include <initializer_list>
#include <utility>

#include "resim/assert/assert.hh"

namespace resim::dynamics::aerodynamics {

namespace {
using transforms::SE3;
using Frame = transforms::Frame<SE3::DIMS>;
using FramedVector = transforms::FramedVector<SE3::DIMS>;
using RigidBodyState = actor::state::RigidBodyState<SE3>;
}  // namespace

AerodynamicElement::AerodynamicElement(SE3 com_from_cop)
    : com_from_cop_(std::move(com_from_cop)) {}

FramedVector AerodynamicElement::cop_local_wind(
    const RigidBodyState &body_com_state,
    const FramedVector &ref_local_wind) const {
  // Start with some frame book-keeping.
  const Frame &ref_frame = body_com_state.ref_from_body().into();
  const Frame &com_frame = body_com_state.ref_from_body().from();
  const Frame &cop_frame = com_from_cop_.from();
  // The body frame of the rigid body state should be the CoM frame of the unit.
  REASSERT(
      com_frame == com_from_cop_.into(),
      "State body frame does not match the center-of-mass frame of the "
      "aerodynamic element");
  // The ref frame of the rigid body state should be the ref frame of the local
  // wind.
  REASSERT(
      ref_frame == ref_local_wind.frame(),
      "The ref frames of the body and the local wind vector should match.");

  // Because this structure is rigid, and we do not allow the CoP to move, the
  // CoM and CoP have no derivatives relative to each other.
  const RigidBodyState com_from_cop_state(com_from_cop_);
  const RigidBodyState ref_from_cop_state = body_com_state * com_from_cop_state;
  // Apparent wind at the CoP is the opposite of the linear velocity of the CoP.
  // Angular velocities at the CoP have no effect.
  const FramedVector apparent_cop_wind(
      -ref_from_cop_state.body_linear_velocity_mps(),
      cop_frame);
  // Put the ambient ref_local_wind in the CoP frame
  const FramedVector ambient_cop_wind =
      ref_from_cop_state.ref_from_body().inverse().rotate(ref_local_wind);
  // Total CoP wind is the sum of the ambient and apparent winds.
  return apparent_cop_wind + ambient_cop_wind;
}

FramedVector AerodynamicElement::cop_local_force(
    const RigidBodyState &body_com_state,
    const FramedVector &ref_local_wind,
    const AerodynamicElementState &aerodynamic_state) const {
  return aerodynamics_(
      cop_local_wind(body_com_state, ref_local_wind),
      aerodynamic_state);
}

typename SE3::TangentVector AerodynamicElement::com_force(
    const RigidBodyState &body_com_state,
    const FramedVector &ref_local_wind,
    const AerodynamicElementState &aerodynamic_state) const {
  const FramedVector cop_force =
      cop_local_force(body_com_state, ref_local_wind, aerodynamic_state);

  // Pressure force acts at the centre of pressure, so there is no torque.
  // TODO(tknowles): Add torque support here, and move from using
  // centers of pressure to aerodynamic centers.
  const FramedVector cop_torque =
      FramedVector(Eigen::Vector3d::Zero(), cop_force.frame());
  const SE3::TangentVector cop_joint_force =
      SE3::tangent_vector_from_parts(cop_torque, cop_force);
  return com_from_cop_.inverse().adjoint().transpose() * cop_joint_force;
}

const SE3 &AerodynamicElement::com_from_cop() const { return com_from_cop_; }
const Frame &AerodynamicElement::com_frame() const {
  return com_from_cop_.into();
}
const Frame &AerodynamicElement::cop_frame() const {
  return com_from_cop_.from();
}

RigidBodyAerodynamics::RigidBodyAerodynamics(
    std::vector<std::shared_ptr<AerodynamicElement>> &&components)
    : components_(std::move(components)) {}

RigidBodyAerodynamics::RigidBodyAerodynamics(
    std::initializer_list<std::shared_ptr<AerodynamicElement>> components) {
  for (const auto &component : components) {
    append(component);
  }
}

void RigidBodyAerodynamics::append(
    std::shared_ptr<AerodynamicElement> component) {
  if (!components_.empty()) {
    constexpr auto FRAME_ERR =
        "All components must have the same center-of-mass (CoM) frame";
    REASSERT(component->com_frame() == this->com_frame(), FRAME_ERR);
  }
  components_.push_back(std::move(component));
}

typename SE3::TangentVector RigidBodyAerodynamics::body_pressure_force(
    const RigidBodyState &body_com_state,
    const FramedVector &ref_local_wind,
    const std::vector<std::reference_wrapper<const AerodynamicElementState>>
        &component_states) const {
  TangentVector pressure_force = TangentVector::Zero();

  REASSERT(
      components_.size() == component_states.size(),
      "Each aerodynamic component must have exactly one associated element "
      "state.");
  for (size_t i = 0; i < components_.size(); ++i) {
    pressure_force += components_[i]->com_force(
        body_com_state,
        ref_local_wind,
        component_states.at(i));
  }
  return pressure_force;
}

const std::vector<std::shared_ptr<AerodynamicElement>>
    &RigidBodyAerodynamics::components() const {
  return components_;
}

const Frame &RigidBodyAerodynamics::com_frame() const {
  return components_.front()->com_frame();
}

}  // namespace resim::dynamics::aerodynamics
