#pragma once

#include "transforms/frame.hh"
#include "transforms/liegroup_concepts.hh"

namespace resim::curves {

// TwoJet
//
// Based on the LieGroups library and LieSpline documentation available at:
// https://ethaneade.com/liespline.pdf
//
// For describing a point on a time parameterized curve (trajectory) with
// Group::DOF degrees of freedom, where Group is a LieGroup class.
//
// The Group data member of the TwoJet<Group> class describes a rigid transform
// between a reference frame and the frame of the point.
//
// Additional data members are TangentVectors specifying the first and second
// time derivatives, thus describing the motion of the point. These derivatives
// are in *LEFT* tangent space.
//
// Note that the transform describing the position/pose of the point is
// frame_from_ref as opposed to ref_from_frame - which is conventional elsewhere
// in this codebase. The departure from convention is because time derivatives
// are in Left tangent space and derivatives in the frame of the point are a
// little easier to work with.
template <transforms::LieGroupType Group>
class TwoJet {
 public:
  using TangentVector = typename Group::TangentVector;
  TwoJet() = default;
  // Construct a TwoJet by providing data members.
  TwoJet(
      Group frame_from_ref,
      TangentVector d_frame_from_ref,
      TangentVector d2_frame_from_ref);

  // Set Data members directly
  void set_frame_from_ref(Group frame_from_ref);
  void set_d_frame_from_ref(TangentVector d_frame_from_ref);
  void set_d2_frame_from_ref(TangentVector d2_frame_from_ref);

  // Get and identity TwoJet.
  static TwoJet<Group> identity();

  // Invert this TwoJet.
  TwoJet<Group> inverse() const;

  // Compose this TwoJet with another. TwoJet composition behaves very similarly
  // to LieGroup composition. It in non-commutative and inner-frames must match
  // for a valid composition. This is enforced if a FramedGroup is used.
  // Derivatives are summed in the left tangent space of the left (this) Group.
  TwoJet<Group> operator*(const TwoJet<Group> &other) const;

  // Access the data members.
  const Group &frame_from_ref() const;
  const TangentVector &d_frame_from_ref() const;
  const TangentVector &d2_frame_from_ref() const;

  bool is_approx(const TwoJet<Group> &other) const;

 private:
  // A rigid transformation describing the position and pose of a frame
  // representing the point relative to a reference frame.
  Group frame_from_ref_;
  // The first time-derivative of the frame_from_ref transform in LEFT tangent
  // space.
  TangentVector d_frame_from_ref_{TangentVector::Zero()};
  // The second time-derivative of the frame_from_ref transform in LEFT tangent
  // space.
  TangentVector d2_frame_from_ref_{TangentVector::Zero()};
};

}  // namespace resim::curves
