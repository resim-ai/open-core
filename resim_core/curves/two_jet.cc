#include "resim_core/curves/two_jet.hh"

#include <utility>

#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::curves {

template <transforms::LieGroupType Group>
TwoJet<Group>::TwoJet(
    Group frame_from_ref,
    TangentVector d_frame_from_ref,
    TangentVector d2_frame_from_ref)
    : frame_from_ref_(std::move(frame_from_ref)),
      d_frame_from_ref_(std::move(d_frame_from_ref)),
      d2_frame_from_ref_(std::move(d2_frame_from_ref)) {}

template <transforms::LieGroupType Group>
void TwoJet<Group>::set_frame_from_ref(Group frame_from_ref) {
  frame_from_ref_ = std::move(frame_from_ref);
}

template <transforms::LieGroupType Group>
void TwoJet<Group>::set_d_frame_from_ref(TangentVector d_frame_from_ref) {
  d_frame_from_ref_ = std::move(d_frame_from_ref);
}

template <transforms::LieGroupType Group>
void TwoJet<Group>::set_d2_frame_from_ref(TangentVector d2_frame_from_ref) {
  d2_frame_from_ref_ = std::move(d2_frame_from_ref);
}

template <transforms::LieGroupType Group>
TwoJet<Group> TwoJet<Group>::identity() {
  return TwoJet<Group>(
      Group::identity(),
      Group::TangentVector::Zero(),
      Group::TangentVector::Zero());
}

template <transforms::LieGroupType Group>
TwoJet<Group> TwoJet<Group>::inverse() const {
  TwoJet<Group> inv = TwoJet<Group>::identity();
  inv.set_frame_from_ref(frame_from_ref_.inverse());
  inv.set_d_frame_from_ref(
      -inv.frame_from_ref().adjoint_times(d_frame_from_ref_));
  inv.set_d2_frame_from_ref(
      -inv.frame_from_ref().adjoint_times(d2_frame_from_ref_));
  return inv;
}

template <transforms::LieGroupType Group>
TwoJet<Group> TwoJet<Group>::operator*(const TwoJet<Group> &other) const {
  return TwoJet<Group>(
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
const Group &TwoJet<Group>::frame_from_ref() const {
  return frame_from_ref_;
}

template <transforms::LieGroupType Group>
const typename Group::TangentVector &TwoJet<Group>::d_frame_from_ref() const {
  return d_frame_from_ref_;
}

template <transforms::LieGroupType Group>
const typename Group::TangentVector &TwoJet<Group>::d2_frame_from_ref() const {
  return d2_frame_from_ref_;
}

template <transforms::LieGroupType Group>
bool TwoJet<Group>::is_approx(const TwoJet<Group> &other) const {
  return (
      frame_from_ref_.is_approx(other.frame_from_ref_) &&
      d_frame_from_ref_.isApprox(other.d_frame_from_ref_) &&
      d2_frame_from_ref_.isApprox(other.d2_frame_from_ref_));
}

template class TwoJet<transforms::SE3>;
template class TwoJet<transforms::SO3>;
template class TwoJet<transforms::FSE3>;
template class TwoJet<transforms::FSO3>;

}  // namespace resim::curves
