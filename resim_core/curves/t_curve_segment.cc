#include "t_curve_segment.hh"

#include <glog/logging.h>

#include <utility>
#include <vector>

#include "resim_core/curves/quintic_poly_coeffs.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_group_concept.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/utils/inout.hh"

namespace resim::curves {

template <transforms::LieGroupType Group>
TCurveSegment<Group>::TCurveSegment(TwoJet<Group> orig, TwoJet<Group> dest)
    : orig_(std::move(orig)),
      dest_(std::move(dest)) {
  if constexpr (transforms::FramedGroupType<Group>) {
    constexpr auto ERROR_MESSAGE =
        "Origin and destination TwoJets must have the same reference frame.";
    const bool frame_equality_test =
        (orig_.frame_from_ref().from() == dest_.frame_from_ref().from());
    CHECK(frame_equality_test) << ERROR_MESSAGE;
  }
}

template <transforms::LieGroupType Group>
TwoJet<Group> TCurveSegment<Group>::point_at(
    const double time_nrm,
    const Frame &point_frame) const {
  TwoJet<Group> point = TwoJet<Group>::identity();
  if constexpr (transforms::FramedGroupType<Group>) {
    // TODO(https://app.asana.com/0/0/1202833644049385/f)
    point.set_frame_from_ref(
        Group::identity(point_frame, orig_.frame_from_ref().into()));
  }

  // dest_from_orig
  const Group dest_from_orig =
      dest_.frame_from_ref() * orig_.frame_from_ref().inverse();
  const TwoJetPolyCoeffs coeffs_dest_from_orig =
      QuinticPolyCoeffs::dest_from_orig(time_nrm);
  point_value_accumulator(
      time_nrm,
      coeffs_dest_from_orig,
      dest_from_orig.log(),
      InOut(point));
  // d_orig
  const TwoJetPolyCoeffs coeffs_d_orig = QuinticPolyCoeffs::d_orig(time_nrm);
  point_value_accumulator(
      time_nrm,
      coeffs_d_orig,
      orig_.d_frame_from_ref(),
      InOut(point));
  // d_dest
  const TwoJetPolyCoeffs coeffs_d_dest = QuinticPolyCoeffs::d_dest(time_nrm);
  point_value_accumulator(
      time_nrm,
      coeffs_d_dest,
      dest_.d_frame_from_ref(),
      InOut(point));
  // d2_orig
  const TwoJetPolyCoeffs coeffs_d2_orig = QuinticPolyCoeffs::d2_orig(time_nrm);
  point_value_accumulator(
      time_nrm,
      coeffs_d2_orig,
      orig_.d2_frame_from_ref(),
      InOut(point));
  // d2_dest
  const TwoJetPolyCoeffs coeffs_d2_dest = QuinticPolyCoeffs::d2_dest(time_nrm);
  point_value_accumulator(
      time_nrm,
      coeffs_d2_dest,
      dest_.d2_frame_from_ref(),
      InOut(point));

  // Reframe the point from the origin to the curve segment reference.
  point.set_frame_from_ref(point.frame_from_ref() * orig_.frame_from_ref());
  return point;
}

template <transforms::LieGroupType Group>
const TwoJet<Group> &TCurveSegment<Group>::orig() const {
  return orig_;
}

template <transforms::LieGroupType Group>
const TwoJet<Group> &TCurveSegment<Group>::dest() const {
  return dest_;
}

template <transforms::LieGroupType Group>
void TCurveSegment<Group>::point_value_accumulator(
    const double time_nrm,
    const TwoJetPolyCoeffs &coeffs,
    const TangentVector &vec,
    InOut<TwoJet<Group>> point) const {
  TwoJet<Group> increment(
      increment_group(coeffs.a * vec, *point),
      coeffs.da * vec,
      coeffs.d2a * vec);

  *point = increment * *point;
}

template <transforms::LieGroupType Group>
Group TCurveSegment<Group>::increment_group(
    const TangentVector &alg,
    const TwoJet<Group> &point) const {
  Group increment = Group::identity();
  if constexpr (transforms::FramedGroupType<Group>) {
    const Frame &point_frame = point.frame_from_ref().into();
    increment = Group::exp(alg, point_frame, point_frame);
  } else {
    increment = Group::exp(alg);
  }
  return increment;
}

template class TCurveSegment<transforms::SO3>;
template class TCurveSegment<transforms::SE3>;
template class TCurveSegment<transforms::FSO3>;
template class TCurveSegment<transforms::FSE3>;

}  // namespace resim::curves
