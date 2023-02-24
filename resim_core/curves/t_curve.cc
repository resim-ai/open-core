#include "resim_core/curves/t_curve.hh"

#include <glog/logging.h>

#include <algorithm>

#include "resim_core/curves/two_jet.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_group_concept.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/utils/inout.hh"

namespace resim::curves {

namespace {
using SE3 = transforms::SE3;
using SO3 = transforms::SO3;
using FSE3 = transforms::FSE3;
using FSO3 = transforms::FSO3;

constexpr auto EMPTY_ERR =
    "Cannot query the frame of a curve with no control points";

template <transforms::FramedGroupType Group>
const transforms::Frame<Group::DIMS> &reference_frame_impl(
    const std::vector<typename TCurve<Group>::Control> &control_pts) {
  CHECK(!control_pts.empty()) << EMPTY_ERR;
  return control_pts.back().point.frame_from_ref().from();
}

template <transforms::FramedGroupType Group>
const transforms::Frame<Group::DIMS> &point_frame_impl(
    const std::vector<typename TCurve<Group>::Control> &control_pts) {
  CHECK(!control_pts.empty()) << EMPTY_ERR;
  return control_pts.back().point.frame_from_ref().into();
}
}  // namespace

template <transforms::LieGroupType Group>
TCurve<Group>::TCurve(const std::vector<Control> &points) {
  for (const auto &point : points) {
    append(point);
  }
}

template <transforms::LieGroupType Group>
TCurve<Group>::TCurve(std::initializer_list<Control> points) {
  append(points);
}

template <transforms::LieGroupType Group>
void TCurve<Group>::append(Control point) {
  if (control_pts_.empty()) {
    control_pts_.push_back(std::move(point));
  } else {
    // Check that time is increasing.
    constexpr auto TIME_ERR =
        "Control points must have strictly increasing time";
    CHECK(point.time > control_pts_.back().time) << TIME_ERR;
    // Check that both control point frames match.
    if constexpr (transforms::FramedGroupType<Group>) {
      constexpr auto FRAME_ERR =
          "Control points must all have the same ref and point frame";
      const bool frame_test = point.point.frame_from_ref().verify_frames(
          this->point_frame(),
          this->reference_frame());
      CHECK(frame_test) << FRAME_ERR;
    }
    // Build a segment, note segments are time normalized so we need
    // to create new control points with scaled derivatives.
    const double dt = point.time - control_pts_.back().time;
    const TwoJetL<Group> orig(
        control_pts_.back().point.frame_from_ref(),
        control_pts_.back().point.d_frame_from_ref() * dt,
        control_pts_.back().point.d2_frame_from_ref() * dt * dt);
    const TwoJetL<Group> dest(
        point.point.frame_from_ref(),
        point.point.d_frame_from_ref() * dt,
        point.point.d2_frame_from_ref() * dt * dt);
    segments_.push_back(
        {control_pts_.back().time,
         point.time,
         TCurveSegment<Group>(orig, dest)});
    // Finally, add the control point.
    control_pts_.push_back(std::move(point));
  }
}

template <transforms::LieGroupType Group>
void TCurve<Group>::append(std::initializer_list<Control> points) {
  for (const auto &point : points) {
    append(point);
  }
}

template <>
const transforms::Frame<FSE3::DIMS> &TCurve<FSE3>::reference_frame() const {
  return reference_frame_impl<FSE3>(control_pts_);
}

template <>
const transforms::Frame<FSO3::DIMS> &TCurve<FSO3>::reference_frame() const {
  return reference_frame_impl<FSO3>(control_pts_);
}

template <>
const transforms::Frame<FSE3::DIMS> &TCurve<FSE3>::point_frame() const {
  return point_frame_impl<FSE3>(control_pts_);
}

template <>
const transforms::Frame<FSO3::DIMS> &TCurve<FSO3>::point_frame() const {
  return point_frame_impl<FSO3>(control_pts_);
}

template <transforms::LieGroupType Group>
TwoJetL<Group> TCurve<Group>::point_at(const double time) const {
  const PointAtData pd = point_at_impl(time);
  TwoJetL<Group> point = pd.in_segment.curve.point_at(pd.time_nrm);
  unnormalize_derivatives(pd.inv_dt, InOut(point));
  return point;
}

template <>
TwoJetL<FSE3> TCurve<FSE3>::point_at(const double time) const {
  const PointAtData pd = point_at_impl(time);
  TwoJetL<FSE3> point =
      pd.in_segment.curve.point_at(pd.time_nrm, point_frame());
  unnormalize_derivatives(pd.inv_dt, InOut(point));
  return point;
}

template <>
TwoJetL<FSO3> TCurve<FSO3>::point_at(const double time) const {
  const PointAtData pd = point_at_impl(time);
  TwoJetL<FSO3> point =
      pd.in_segment.curve.point_at(pd.time_nrm, point_frame());
  unnormalize_derivatives(pd.inv_dt, InOut(point));
  return point;
}

template <transforms::LieGroupType Group>
const std::vector<typename TCurve<Group>::Control> &TCurve<Group>::control_pts()
    const {
  return control_pts_;
}

template <transforms::LieGroupType Group>
const std::vector<typename TCurve<Group>::Segment> &TCurve<Group>::segments()
    const {
  return segments_;
}

template <transforms::LieGroupType Group>
typename TCurve<Group>::PointAtData TCurve<Group>::point_at_impl(
    const double time) const {
  constexpr auto EMPTY_ERR =
      "Cannot query a curve with less than two control points";
  CHECK(!segments_.empty()) << EMPTY_ERR;
  constexpr auto TIME_RANGE_ERR =
      "Query time must be within the range of times in the control points.";
  const bool time_lo = time < control_pts_.front().time;
  const bool time_hi = time > control_pts_.back().time;
  CHECK(!time_lo && !time_hi) << TIME_RANGE_ERR;
  const auto s = std::lower_bound(
      segments_.begin(),
      segments_.end(),
      time,
      [](const Segment &segment, double time) {
        return segment.dest_time < time;
      });
  const double dt = s->dest_time - s->orig_time;
  const double inv_dt = 1. / dt;
  const double time_nrm = inv_dt * (time - s->orig_time);
  return {*s, inv_dt, time_nrm};
}

template <transforms::LieGroupType Group>
void TCurve<Group>::unnormalize_derivatives(
    const double inv_dt,
    InOut<TwoJetL<Group>> point) const {
  point->set_d_frame_from_ref(point->d_frame_from_ref() * inv_dt);
  point->set_d2_frame_from_ref(point->d2_frame_from_ref() * inv_dt * inv_dt);
}

template <transforms::LieGroupType Group>
double TCurve<Group>::end_time() const {
  CHECK(!control_pts_.empty()) << EMPTY_ERR;
  return control_pts_.back().time;
}

template <transforms::LieGroupType Group>
double TCurve<Group>::start_time() const {
  CHECK(!control_pts_.empty()) << EMPTY_ERR;
  return control_pts_.front().time;
}

template class TCurve<FSO3>;
template class TCurve<FSE3>;
template class TCurve<SE3>;
template class TCurve<SO3>;

}  // namespace resim::curves
