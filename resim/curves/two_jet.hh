#pragma once

#include "resim/curves/two_jet_concepts.hh"
#include "resim/math/is_approx.hh"
#include "resim/transforms/liegroup_concepts.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves {

// TwoJet
//
// Based on the LieGroups library and LieSpline documentation available at:
// https://ethaneade.com/liespline.pdf
//
// For describing a point on a time parameterized curve (trajectory) with
// Group::DOF degrees of freedom, where Group is a LieGroup class.
//
// The Group data member of the TwoJet describes a rigid transform between a
// reference frame and the frame of the point.
//
// Additional data members are TangentVectors specifying the first and second
// time derivatives, thus describing the motion of the transform.

// These derivatives may be expressed in either *LEFT*  or *RIGHT* tangent
// space. To make this as explicit as possible we provide two flavors of TwoJet
// below. TwoJetL - with the point frame on the left and derivatives in LEFT
// tangent space. But, also TwoJetR - with the point frame on the right and
// derivatives in RIGHT tangent space. For more information on left and right
// tangent spaces, see the following documentation:
//  - https://docs.resim.ai/curves/#two-jet
//  - https://docs.resim.ai/transforms/using_liegroups/
//  - https://docs.resim.ai/transforms/liegroup_derivatives/
//
// In general we expect implementations to be opinionated about whether they
// prefer to be in LEFT or RIGHT tangent space and use either TwoJetL or
// TwoJetR, but not both. Instead, relying on the fact that it easy to convert
// between these with the methods provided.

// Forward declare TwoJetR so that TwoJetL can define conversion to right space
// methods e.g. TwoJetR<Group> right_two_jet().
template <transforms::LieGroupType Group>
class TwoJetR;

// TwoJetL
//
// A left-tangent-space version of a TwoJet.
//
// The Group data member of the TwoJetL<Group> is a LieGroup class describing
// a rigid transform from a reference frame to the frame of the point. That is,
// the point frame is on the left (frame_from ref)
//
// Additional data members are TangentVectors specifying the first and second
// time derivatives, thus describing the motion of the point. These derivatives
// are in *LEFT* tangent space.
//
template <transforms::LieGroupType Group>
class TwoJetL {
 public:
  using GroupType = Group;
  using TangentVector = typename Group::TangentVector;
  TwoJetL() = default;
  // Construct a TwoJetL by providing data members.
  TwoJetL(
      Group frame_from_ref,
      TangentVector d_frame_from_ref,
      TangentVector d2_frame_from_ref);

  // Set Data members directly
  void set_frame_from_ref(Group frame_from_ref);
  void set_d_frame_from_ref(TangentVector d_frame_from_ref);
  void set_d2_frame_from_ref(TangentVector d2_frame_from_ref);

  // Get an identity TwoJetL.
  template <typename... Args>
  static TwoJetL<Group> identity(Args &&...args);

  // Invert this TwoJetL.
  TwoJetL<Group> inverse() const;

  // Compose this TwoJetL with another. TwoJetL composition behaves very
  // similarly to LieGroup composition. It is non-commutative and inner-frames
  // must match for a valid composition. Derivatives are summed in the left
  // tangent space of the left (this) Group.
  TwoJetL<Group> operator*(const TwoJetL<Group> &other) const;

  // Convert this two jet to a right two jet. This library is opinionated about
  // the frame ordering in right and left two jets. Namely in a right two jet
  // the point frame is on the right and in a left two jet the point frame is on
  // the left. Thus this method inverts the transform *AND* moves the
  // derivatives from left  to right tangent space.
  TwoJetR<Group> right_two_jet() const;

  // Access the data members.
  const Group &frame_from_ref() const;
  const TangentVector &d_frame_from_ref() const;
  const TangentVector &d2_frame_from_ref() const;

  bool is_approx(
      const TwoJetL<Group> &other,
      double precision = math::DEFAULT_PRECISION) const;

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

static_assert(
    TwoJetType<TwoJetL<transforms::SE3>>,
    "TwoJetL doesn't meet the requirements for a two jet.");

// TwoJetR
//
// A right-tangent-space version of a TwoJet.
//
// The Group data member of the TwoJetR<Group> is a LieGroup class describing
// a rigid transform from the point frame to a reference frame. That is,
// the point frame is on the right (ref_from_frame).
//
// Additional data members are TangentVectors specifying the first and second
// time derivatives, thus describing the motion of the point. These derivatives
// are in *RIGHT* tangent space.
//
template <transforms::LieGroupType Group>
class TwoJetR {
 public:
  using GroupType = Group;
  using TangentVector = typename Group::TangentVector;
  TwoJetR() = default;
  // Construct a TwoJetR by providing data members.
  TwoJetR(
      Group ref_from_frame,
      TangentVector d_ref_from_frame,
      TangentVector d2_ref_from_frame);

  // Set Data members directly
  void set_ref_from_frame(Group ref_from_frame);
  void set_d_ref_from_frame(TangentVector d_ref_from_frame);
  void set_d2_ref_from_frame(TangentVector d2_ref_from_frame);

  // Get an identity TwoJetR.
  template <typename... Args>
  static TwoJetR<Group> identity(Args &&...args);

  // Invert this TwoJetR.
  TwoJetR<Group> inverse() const;

  // Compose this TwoJetR with another. TwoJetR composition behaves very
  // similarly to LieGroup composition. It is non-commutative and inner-frames
  // must match for a valid composition. Derivatives are summed in the right
  // tangent space of the left (this) Group.
  TwoJetR<Group> operator*(const TwoJetR<Group> &other) const;

  // Convert this two jet to a left two jet. This library is opinionated about
  // the frame ordering in right and left two jets. Namely in a right two jet
  // the point frame is on the right and in a left two jet the point frame is on
  // the left. Thus this method inverts the transform *AND* moves the
  // derivatives from right to left tangent space.
  TwoJetL<Group> left_two_jet() const;

  // Access the data members.
  const Group &ref_from_frame() const;
  const TangentVector &d_ref_from_frame() const;
  const TangentVector &d2_ref_from_frame() const;

  bool is_approx(
      const TwoJetR<Group> &other,
      double precision = math::DEFAULT_PRECISION) const;

 private:
  // A rigid transformation describing the position and pose of a frame
  // representing the point relative to a reference frame.
  Group ref_from_frame_;
  // The first time-derivative of the ref_from_frame transform in RIGHT tangent
  // space.
  TangentVector d_ref_from_frame_{TangentVector::Zero()};
  // The second time-derivative of the ref_from_frame transform in RIGHT tangent
  // space.
  TangentVector d2_ref_from_frame_{TangentVector::Zero()};
};

static_assert(
    TwoJetType<TwoJetR<transforms::SE3>>,
    "TwoJetR doesn't meet the requirements for a two jet.");

}  // namespace resim::curves
