#include "resim_core/actor/state/rigid_body_state.hh"

#include <type_traits>
#include <utility>

#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::actor::state {

template <transforms::LieGroupType Group>
using LinearDelta = typename RigidBodyState<Group>::LinearDelta;

template <transforms::LieGroupType Group>
using LinearDeltaBlock = typename RigidBodyState<Group>::LinearDeltaBlock;

template <transforms::LieGroupType Group>
using AngularDelta = typename RigidBodyState<Group>::AngularDelta;

template <transforms::LieGroupType Group>
using AngularDeltaBlock = typename RigidBodyState<Group>::AngularDeltaBlock;

template <transforms::LieGroupType Group>
using StateDerivatives = typename RigidBodyState<Group>::StateDerivatives;

template <transforms::LieGroupType Group>
RigidBodyState<Group>::RigidBodyState(Group ref_from_body) {
  this->set_ref_from_body(std::move(ref_from_body));
};

template <transforms::LieGroupType Group>
RigidBodyState<Group>::RigidBodyState(
    Group ref_from_body,
    StateDerivatives body_derivatives)
    : ref_from_body_(
          std::move(ref_from_body),
          Group::tangent_vector_from_parts(
              body_derivatives.velocity.angular_radps,
              body_derivatives.velocity.linear_mps),
          Group::tangent_vector_from_parts(
              body_derivatives.acceleration.angular_radpss,
              body_derivatives.acceleration.linear_mpss)) {}

template <transforms::LieGroupType Group>
RigidBodyState<Group>::RigidBodyState(curves::TwoJetR<Group> ref_from_body)
    : ref_from_body_{std::move(ref_from_body)} {};

template <transforms::LieGroupType Group>
template <typename... Args>
RigidBodyState<Group> RigidBodyState<Group>::identity(Args &&...args) {
  return RigidBodyState{
      curves::TwoJetR<Group>::identity(std::forward<Args>(args)...)};
}

template <transforms::LieGroupType Group>
RigidBodyState<Group> RigidBodyState<Group>::operator*(
    const RigidBodyState<Group> &other) const {
  return RigidBodyState{ref_from_body_ * other.ref_from_body_};
}

template <transforms::LieGroupType Group>
RigidBodyState<Group> RigidBodyState<Group>::inverse_times(
    const RigidBodyState<Group> &other) const {
  return RigidBodyState{ref_from_body_.inverse() * other.ref_from_body_};
}

template <transforms::LieGroupType Group>
const Group &RigidBodyState<Group>::ref_from_body() const {
  return ref_from_body_.ref_from_frame();
}

template <transforms::LieGroupType Group>
StateDerivatives<Group> RigidBodyState<Group>::body_derivatives() const {
  return {
      .velocity =
          {.linear_mps = body_linear_velocity_mps(),
           .angular_radps = body_angular_velocity_radps()},
      .acceleration = {
          .linear_mpss = body_linear_acceleration_mpss(),
          .angular_radpss = body_angular_acceleration_radpss()}};
}

template <transforms::LieGroupType Group>
LinearDeltaBlock<Group> RigidBodyState<Group>::body_linear_velocity_mps()
    const {
  return Group::tangent_vector_translation_part(
      ref_from_body_.d_ref_from_frame());
}

template <transforms::LieGroupType Group>
AngularDeltaBlock<Group> RigidBodyState<Group>::body_angular_velocity_radps()
    const {
  return Group::tangent_vector_rotation_part(ref_from_body_.d_ref_from_frame());
}

template <transforms::LieGroupType Group>
LinearDeltaBlock<Group> RigidBodyState<Group>::body_linear_acceleration_mpss()
    const {
  return Group::tangent_vector_translation_part(
      ref_from_body_.d2_ref_from_frame());
}

template <transforms::LieGroupType Group>
AngularDeltaBlock<Group>
RigidBodyState<Group>::body_angular_acceleration_radpss() const {
  return Group::tangent_vector_rotation_part(
      ref_from_body_.d2_ref_from_frame());
}

template <transforms::LieGroupType Group>
const curves::TwoJetR<Group> &RigidBodyState<Group>::ref_from_body_two_jet()
    const {
  return ref_from_body_;
}

template <transforms::LieGroupType Group>
void RigidBodyState<Group>::set_ref_from_body(Group ref_from_body) {
  ref_from_body_.set_ref_from_frame(std::move(ref_from_body));
}

template <transforms::LieGroupType Group>
void RigidBodyState<Group>::set_body_linear_velocity_mps(
    LinearDelta body_linear_velocity_mps) {
  typename Group::TangentVector d_ref_from_frame{
      ref_from_body_.d_ref_from_frame()};
  Group::tangent_vector_translation_part(d_ref_from_frame) =
      body_linear_velocity_mps;
  ref_from_body_.set_d_ref_from_frame(d_ref_from_frame);
}

template <transforms::LieGroupType Group>
void RigidBodyState<Group>::set_body_angular_velocity_radps(
    AngularDelta body_angular_velocity_radps) {
  typename Group::TangentVector d_ref_from_frame{
      ref_from_body_.d_ref_from_frame()};
  Group::tangent_vector_rotation_part(d_ref_from_frame) =
      body_angular_velocity_radps;
  ref_from_body_.set_d_ref_from_frame(d_ref_from_frame);
}

template <transforms::LieGroupType Group>
void RigidBodyState<Group>::set_body_linear_acceleration_mpss(
    LinearDelta body_linear_acceleration_mpss) {
  typename Group::TangentVector d2_ref_from_frame{
      ref_from_body_.d2_ref_from_frame()};
  Group::tangent_vector_translation_part(d2_ref_from_frame) =
      body_linear_acceleration_mpss;
  ref_from_body_.set_d2_ref_from_frame(d2_ref_from_frame);
}

template <transforms::LieGroupType Group>
void RigidBodyState<Group>::set_body_angular_acceleration_radpss(
    AngularDelta body_angular_acceleration_radpss) {
  typename Group::TangentVector d2_ref_from_frame{
      ref_from_body_.d2_ref_from_frame()};
  Group::tangent_vector_rotation_part(d2_ref_from_frame) =
      body_angular_acceleration_radpss;
  ref_from_body_.set_d2_ref_from_frame(d2_ref_from_frame);
}

template class RigidBodyState<transforms::FSE3>;
template RigidBodyState<transforms::FSE3>
RigidBodyState<transforms::FSE3>::identity();
template RigidBodyState<transforms::FSE3>
RigidBodyState<transforms::FSE3>::identity(
    const transforms::Frame<transforms::FSE3::DIMS> &,
    const transforms::Frame<transforms::FSE3::DIMS> &);
template class RigidBodyState<transforms::SE3>;
template RigidBodyState<transforms::SE3>
RigidBodyState<transforms::SE3>::identity();

}  // namespace resim::actor::state
