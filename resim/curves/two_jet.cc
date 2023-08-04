// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/two_jet.hh"

#include <cstdio>
#include <utility>

#include "resim/transforms/liegroup_concepts.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves {

template <transforms::LieGroupType Group>
TwoJetL<Group>::TwoJetL(
    Group frame_from_ref,
    TangentVector d_frame_from_ref,
    TangentVector d2_frame_from_ref)
    : frame_from_ref_(std::move(frame_from_ref)),
      d_frame_from_ref_(std::move(d_frame_from_ref)),
      d2_frame_from_ref_(std::move(d2_frame_from_ref)) {}

template <transforms::LieGroupType Group>
void TwoJetL<Group>::set_frame_from_ref(Group frame_from_ref) {
  frame_from_ref_ = std::move(frame_from_ref);
}

template <transforms::LieGroupType Group>
void TwoJetL<Group>::set_d_frame_from_ref(TangentVector d_frame_from_ref) {
  d_frame_from_ref_ = std::move(d_frame_from_ref);
}

template <transforms::LieGroupType Group>
void TwoJetL<Group>::set_d2_frame_from_ref(TangentVector d2_frame_from_ref) {
  d2_frame_from_ref_ = std::move(d2_frame_from_ref);
}

template <transforms::LieGroupType Group>
template <typename... Args>
TwoJetL<Group> TwoJetL<Group>::identity(Args &&...args) {
  return TwoJetL<Group>(
      Group::identity(std::forward<Args>(args)...),
      Group::TangentVector::Zero(),
      Group::TangentVector::Zero());
}

template <transforms::LieGroupType Group>
TwoJetL<Group> TwoJetL<Group>::inverse() const {
  TwoJetL<Group> inv = TwoJetL<Group>::identity();
  inv.set_frame_from_ref(frame_from_ref_.inverse());
  inv.set_d_frame_from_ref(
      -inv.frame_from_ref().adjoint_times(d_frame_from_ref_));
  inv.set_d2_frame_from_ref(
      -inv.frame_from_ref().adjoint_times(d2_frame_from_ref_));
  return inv;
}

template <transforms::LieGroupType Group>
TwoJetL<Group> TwoJetL<Group>::operator*(const TwoJetL<Group> &other) const {
  return TwoJetL<Group>(
      frame_from_ref_ * other.frame_from_ref_,
      d_frame_from_ref_ +
          frame_from_ref_.adjoint_times(other.d_frame_from_ref_),
      d2_frame_from_ref_ +
          frame_from_ref_.adjoint_times(other.d2_frame_from_ref_) +
          Group::adjoint_times(
              d_frame_from_ref_,
              frame_from_ref_.adjoint_times(other.d_frame_from_ref_)));
}

template <transforms::LieGroupType Group>
TwoJetR<Group> TwoJetL<Group>::right_two_jet() const {
  return TwoJetR<Group>(
      frame_from_ref_.inverse(),
      -d_frame_from_ref_,
      -d2_frame_from_ref_);
}

template <transforms::LieGroupType Group>
const Group &TwoJetL<Group>::frame_from_ref() const {
  return frame_from_ref_;
}

template <transforms::LieGroupType Group>
const typename Group::TangentVector &TwoJetL<Group>::d_frame_from_ref() const {
  return d_frame_from_ref_;
}

template <transforms::LieGroupType Group>
const typename Group::TangentVector &TwoJetL<Group>::d2_frame_from_ref() const {
  return d2_frame_from_ref_;
}

template <transforms::LieGroupType Group>
bool TwoJetL<Group>::is_approx(
    const TwoJetL<Group> &other,
    const double precision) const {
  return (
      frame_from_ref_.is_approx(other.frame_from_ref_, precision) &&
      math::is_approx(d_frame_from_ref_, other.d_frame_from_ref_, precision) &&
      math::is_approx(d2_frame_from_ref_, other.d2_frame_from_ref_, precision));
}

template class TwoJetL<transforms::SE3>;
template class TwoJetL<transforms::SO3>;
template TwoJetL<transforms::SE3> TwoJetL<transforms::SE3>::identity(
    const transforms::Frame<transforms::SE3::DIMS> &,
    const transforms::Frame<transforms::SE3::DIMS> &);
template TwoJetL<transforms::SO3> TwoJetL<transforms::SO3>::identity(
    const transforms::Frame<transforms::SO3::DIMS> &,
    const transforms::Frame<transforms::SO3::DIMS> &);

template <transforms::LieGroupType Group>
TwoJetR<Group>::TwoJetR(
    Group ref_from_frame,
    TangentVector d_ref_from_frame,
    TangentVector d2_ref_from_frame)
    : ref_from_frame_(std::move(ref_from_frame)),
      d_ref_from_frame_(std::move(d_ref_from_frame)),
      d2_ref_from_frame_(std::move(d2_ref_from_frame)) {}

template <transforms::LieGroupType Group>
void TwoJetR<Group>::set_ref_from_frame(Group ref_from_frame) {
  ref_from_frame_ = std::move(ref_from_frame);
}

template <transforms::LieGroupType Group>
void TwoJetR<Group>::set_d_ref_from_frame(TangentVector d_ref_from_frame) {
  d_ref_from_frame_ = std::move(d_ref_from_frame);
}

template <transforms::LieGroupType Group>
void TwoJetR<Group>::set_d2_ref_from_frame(TangentVector d2_ref_from_frame) {
  d2_ref_from_frame_ = std::move(d2_ref_from_frame);
}

template <transforms::LieGroupType Group>
template <typename... Args>
TwoJetR<Group> TwoJetR<Group>::identity(Args &&...args) {
  return TwoJetR<Group>(
      Group::identity(std::forward<Args>(args)...),
      Group::TangentVector::Zero(),
      Group::TangentVector::Zero());
}

template <transforms::LieGroupType Group>
TwoJetR<Group> TwoJetR<Group>::inverse() const {
  TwoJetR<Group> inv = TwoJetR<Group>::identity();
  inv.set_ref_from_frame(ref_from_frame_.inverse());
  inv.set_d_ref_from_frame(-ref_from_frame().adjoint_times(d_ref_from_frame_));
  inv.set_d2_ref_from_frame(
      -ref_from_frame().adjoint_times(d2_ref_from_frame_));
  return inv;
}

template <transforms::LieGroupType Group>
TwoJetR<Group> TwoJetR<Group>::operator*(const TwoJetR<Group> &other) const {
  return TwoJetR<Group>(
      ref_from_frame_ * other.ref_from_frame_,
      other.d_ref_from_frame_ +
          other.ref_from_frame_.inverse().adjoint_times(d_ref_from_frame_),
      other.d2_ref_from_frame_ +
          other.ref_from_frame_.inverse().adjoint_times(d2_ref_from_frame_) -
          Group::adjoint_times(
              other.d_ref_from_frame_,
              other.ref_from_frame_.inverse().adjoint_times(
                  d_ref_from_frame_)));
}

template <transforms::LieGroupType Group>
TwoJetL<Group> TwoJetR<Group>::left_two_jet() const {
  return TwoJetL<Group>(
      ref_from_frame_.inverse(),
      -d_ref_from_frame_,
      -d2_ref_from_frame_);
}

template <transforms::LieGroupType Group>
const Group &TwoJetR<Group>::ref_from_frame() const {
  return ref_from_frame_;
}

template <transforms::LieGroupType Group>
const typename Group::TangentVector &TwoJetR<Group>::d_ref_from_frame() const {
  return d_ref_from_frame_;
}

template <transforms::LieGroupType Group>
const typename Group::TangentVector &TwoJetR<Group>::d2_ref_from_frame() const {
  return d2_ref_from_frame_;
}

template <transforms::LieGroupType Group>
bool TwoJetR<Group>::is_approx(
    const TwoJetR<Group> &other,
    const double precision) const {
  return (
      ref_from_frame_.is_approx(other.ref_from_frame_, precision) &&
      math::is_approx(d_ref_from_frame_, other.d_ref_from_frame_, precision) &&
      math::is_approx(d2_ref_from_frame_, other.d2_ref_from_frame_, precision));
}

template class TwoJetR<transforms::SE3>;
template class TwoJetR<transforms::SO3>;
template TwoJetR<transforms::SE3> TwoJetR<transforms::SE3>::identity(
    const transforms::Frame<transforms::SE3::DIMS> &,
    const transforms::Frame<transforms::SE3::DIMS> &);
template TwoJetR<transforms::SO3> TwoJetR<transforms::SO3>::identity(
    const transforms::Frame<transforms::SO3::DIMS> &,
    const transforms::Frame<transforms::SO3::DIMS> &);

}  // namespace resim::curves
