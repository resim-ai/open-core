#include "resim_core/curves/two_jet.hh"

#include <utility>

#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

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
TwoJetL<Group> TwoJetL<Group>::identity() {
  return TwoJetL<Group>(
      Group::identity(),
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
bool TwoJetL<Group>::is_approx(const TwoJetL<Group> &other) const {
  return (
      frame_from_ref_.is_approx(other.frame_from_ref_) &&
      d_frame_from_ref_.isApprox(other.d_frame_from_ref_) &&
      d2_frame_from_ref_.isApprox(other.d2_frame_from_ref_));
}

// TODO(https://app.asana.com/0/1202178773526279/1203608723315721/f)
template class TwoJetL<transforms::SE3>;
template class TwoJetL<transforms::SO3>;
template class TwoJetL<transforms::FSE3>;
template class TwoJetL<transforms::FSO3>;

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
TwoJetR<Group> TwoJetR<Group>::identity() {
  return TwoJetR<Group>(
      Group::identity(),
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
bool TwoJetR<Group>::is_approx(const TwoJetR<Group> &other) const {
  return (
      ref_from_frame_.is_approx(other.ref_from_frame_) &&
      d_ref_from_frame_.isApprox(other.d_ref_from_frame_) &&
      d2_ref_from_frame_.isApprox(other.d2_ref_from_frame_));
}

template class TwoJetR<transforms::SE3>;
template class TwoJetR<transforms::SO3>;
template class TwoJetR<transforms::FSE3>;
template class TwoJetR<transforms::FSO3>;

}  // namespace resim::curves
