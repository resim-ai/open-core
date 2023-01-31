#pragma once

#include "resim_core/curves/quintic_poly_coeffs.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/utils/inout.hh"

namespace resim::curves {

// A Single segment of a time parameterised curve.
// The curve is a quintic Hermite spline in LieGroups based on the documentation
// available at: https://ethaneade.com/lie_spline.pdf. The degrees-of-freedom of
// the curve is determined by the underlying LieGroup used.

// The curve is specified by its boundary conditions at two TwoJet objects
// called - respectively - the origin (orig) and the destination (dest).
// Time in the segment is normalized, without loss of generality. The
// normalized time (time_nrm) at the origin is 0. The time_nrm
// at the destination is 1.
template <transforms::LieGroupType Group>
class TCurveSegment {
 public:
  using Frame = transforms::Frame<Group::DIMS>;
  using TangentVector = typename Group::TangentVector;

  TCurveSegment() = delete;

  // Initialize a curve with origin and destination boundary conditions.
  // If using a FramedGroup, orig and dest must use the same reference frame.
  TCurveSegment(TwoJetL<Group> orig, TwoJetL<Group> dest);

  // Retrieve a point on the curve at normalized time time_nrm.
  // If the curve is in a FramedGroup the user may optionally specify a frame.
  // for the point.
  TwoJetL<Group> point_at(
      double time_nrm,
      const Frame &point_frame = Frame::new_frame()) const;

  const TwoJetL<Group> &orig() const;
  const TwoJetL<Group> &dest() const;

 private:
  // Points are built up incrementally by composing five TwoJets, sequentially
  // with the origin point. Each TwoJet is generated by multiplying appropriate
  // polynomial coefficients by a TangentVector the five TangentVectors are:
  // The orig_from_dest transform in the algebra representation. The two time
  // derivatives of the origin TwoJet and the two time derivatives of the
  // destination TwoJet. Each operation follows the same pattern which is
  // executed in this method.
  // time_nrm - is the normalized time passed down from point_at
  // coeffs are the polynomial coefficients.
  // vec is the TangentVector.
  // point is the point which will be modified within this method.
  void point_value_accumulator(
      double time_nrm,
      const TwoJetPolyCoeffs &coeffs,
      const TangentVector &vec,
      InOut<TwoJetL<Group>> point) const;
  // A helper specifically for incrementing the Group part of the point TwoJet.
  // alg - is the algebra representation of the Group being applied in the
  // increment.
  // point - is the point to which the increment will be applied. The
  // composition does not occur here, however we need a reference to the point
  // for the case when the Group is a FramedGroup and we wish to ensure
  // consistency in the frames used in the increment.
  Group increment_group(const TangentVector &alg, const TwoJetL<Group> &point)
      const;

  TwoJetL<Group> orig_;
  TwoJetL<Group> dest_;
};

}  // namespace resim::curves
