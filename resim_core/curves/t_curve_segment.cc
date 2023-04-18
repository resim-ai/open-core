#include "t_curve_segment.hh"

#include <utility>
#include <vector>

#include "resim_core/assert/assert.hh"
#include "resim_core/curves/quintic_poly_coeffs.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_group_concept.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/utils/inout.hh"

namespace resim::curves {

namespace {
using FSE3 = transforms::FSE3;
using FSO3 = transforms::FSO3;
}  // namespace

template <transforms::LieGroupType Group>
TCurveSegment<Group>::TCurveSegment(TwoJetL<Group> orig, TwoJetL<Group> dest)
    : orig_(std::move(orig)),
      dest_(std::move(dest)) {
  if constexpr (transforms::FramedGroupType<Group>) {
    constexpr auto ERROR_MESSAGE =
        "Origin and destination TwoJets must have matching frames.";
    REASSERT(
        orig_.frame_from_ref().verify_frames(
            dest_.frame_from_ref().into(),
            dest_.frame_from_ref().from()),
        ERROR_MESSAGE);
  }
}

template <transforms::LieGroupType Group>
TwoJetL<Group> TCurveSegment<Group>::point_at(const double time_nrm) const {
  TwoJetL<Group> point = TwoJetL<Group>::identity();
  if constexpr (transforms::FramedGroupType<Group>) {
    // Note that we accumlate the point transform relative to the origin point
    // before composing this with the origin point at the end of this function
    // to get point_from_ref. Therefore, the point starts out as
    // point_from_point.
    // TODO(https://app.asana.com/0/0/1202833644049385/f)
    point.set_frame_from_ref(Group::identity(point_frame(), point_frame()));
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

template <>
const transforms::Frame<FSE3::DIMS> &TCurveSegment<FSE3>::reference_frame()
    const {
  return orig_.frame_from_ref().from();
}

template <>
const transforms::Frame<FSO3::DIMS> &TCurveSegment<FSO3>::reference_frame()
    const {
  return orig_.frame_from_ref().from();
}

template <>
const transforms::Frame<FSE3::DIMS> &TCurveSegment<FSE3>::point_frame() const {
  return orig_.frame_from_ref().into();
}

template <>
const transforms::Frame<FSO3::DIMS> &TCurveSegment<FSO3>::point_frame() const {
  return orig_.frame_from_ref().into();
}

template <transforms::LieGroupType Group>
const TwoJetL<Group> &TCurveSegment<Group>::orig() const {
  return orig_;
}

template <transforms::LieGroupType Group>
const TwoJetL<Group> &TCurveSegment<Group>::dest() const {
  return dest_;
}

template <transforms::LieGroupType Group>
void TCurveSegment<Group>::point_value_accumulator(
    const double time_nrm,
    const TwoJetPolyCoeffs &coeffs,
    const TangentVector &vec,
    InOut<TwoJetL<Group>> point) const {
  TwoJetL<Group> increment(
      increment_group(coeffs.a * vec, *point),
      coeffs.da * vec,
      coeffs.d2a * vec);

  *point = increment * *point;
}

template <transforms::LieGroupType Group>
Group TCurveSegment<Group>::increment_group(
    const TangentVector &alg,
    const TwoJetL<Group> &point) const {
  Group increment = Group::identity();
  if constexpr (transforms::FramedGroupType<Group>) {
    increment = Group::exp(alg, point_frame(), point_frame());
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
