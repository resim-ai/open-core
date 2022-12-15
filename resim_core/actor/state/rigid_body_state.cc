#include "resim_core/actor/state/rigid_body_state.hh"

#include <type_traits>
#include <utility>

#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::actor::state {

template <transforms::LieGroupType Group>
using LinearDelta = typename RigidBodyState<Group>::LinearDelta;

template <transforms::LieGroupType Group>
using AngularDelta = typename RigidBodyState<Group>::AngularDelta;

template <transforms::LieGroupType Group>
RigidBodyState<Group>::RigidBodyState(curves::TwoJet<Group> body_from_reference)
    : body_from_reference_{std::move(body_from_reference)} {};

template <transforms::LieGroupType Group>
RigidBodyState<Group> RigidBodyState<Group>::identity() {
  return RigidBodyState{curves::TwoJet<Group>::identity()};
}

template <transforms::LieGroupType Group>
const Group &RigidBodyState<Group>::body_from_reference() const {
  return body_from_reference_.frame_from_ref();
}

template <transforms::LieGroupType Group>
Group RigidBodyState<Group>::reference_from_body() const {
  return body_from_reference().inverse();
}

template <transforms::LieGroupType Group>
LinearDelta<Group> RigidBodyState<Group>::position_m() const {
  return body_from_reference_.frame_from_ref().inverse().translation();
}

template <transforms::LieGroupType Group>
AngularDelta<Group> RigidBodyState<Group>::orientation_rad() const {
  return -body_from_reference_.frame_from_ref().rotation().log();
}

template <transforms::LieGroupType Group>
LinearDelta<Group> RigidBodyState<Group>::linear_velocity_mps() const {
  return -Group::tangent_vector_translation_part(
      body_from_reference_.d_frame_from_ref());
}
template <transforms::LieGroupType Group>
AngularDelta<Group> RigidBodyState<Group>::angular_velocity_radps() const {
  return -Group::tangent_vector_rotation_part(
      body_from_reference_.d_frame_from_ref());
}

template <transforms::LieGroupType Group>
LinearDelta<Group> RigidBodyState<Group>::linear_acceleration_mpss() const {
  return -Group::tangent_vector_translation_part(
      body_from_reference_.d2_frame_from_ref());
}

template <transforms::LieGroupType Group>
AngularDelta<Group> RigidBodyState<Group>::angular_acceleration_radpss() const {
  return -Group::tangent_vector_rotation_part(
      body_from_reference_.d2_frame_from_ref());
}

template <transforms::LieGroupType Group>
const curves::TwoJet<Group>
    &RigidBodyState<Group>::body_from_reference_two_jet() const {
  return body_from_reference_;
}

template <transforms::LieGroupType Group>
void RigidBodyState<Group>::set_reference_from_body(
    const Group &reference_from_body) {
  body_from_reference_.set_frame_from_ref(reference_from_body.inverse());
}

template <transforms::LieGroupType Group>
void RigidBodyState<Group>::set_linear_velocity_mps(
    LinearDelta linear_velocity_mps) {
  typename Group::TangentVector d_frame_from_ref{
      body_from_reference_.d_frame_from_ref()};
  Group::tangent_vector_translation_part(d_frame_from_ref) =
      -linear_velocity_mps;
  body_from_reference_.set_d_frame_from_ref(d_frame_from_ref);
}

template <transforms::LieGroupType Group>
void RigidBodyState<Group>::set_angular_velocity_radps(
    AngularDelta angular_velocity_radps) {
  typename Group::TangentVector d_frame_from_ref{
      body_from_reference_.d_frame_from_ref()};
  Group::tangent_vector_rotation_part(d_frame_from_ref) =
      -angular_velocity_radps;
  body_from_reference_.set_d_frame_from_ref(d_frame_from_ref);
}

template <transforms::LieGroupType Group>
void RigidBodyState<Group>::set_linear_acceleration_mpss(
    LinearDelta linear_acceleration_mpss) {
  typename Group::TangentVector d2_frame_from_ref{
      body_from_reference_.d2_frame_from_ref()};
  Group::tangent_vector_translation_part(d2_frame_from_ref) =
      -linear_acceleration_mpss;
  body_from_reference_.set_d2_frame_from_ref(d2_frame_from_ref);
}

template <transforms::LieGroupType Group>
void RigidBodyState<Group>::set_angular_acceleration_radpss(
    AngularDelta angular_acceleration_radpss) {
  typename Group::TangentVector d2_frame_from_ref{
      body_from_reference_.d2_frame_from_ref()};
  Group::tangent_vector_rotation_part(d2_frame_from_ref) =
      -angular_acceleration_radpss;
  body_from_reference_.set_d2_frame_from_ref(d2_frame_from_ref);
}

template class RigidBodyState<transforms::SE3>;
template class RigidBodyState<transforms::FSE3>;

}  // namespace resim::actor::state
