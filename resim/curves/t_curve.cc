// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/t_curve.hh"

#include <algorithm>

#include "resim/assert/assert.hh"
#include "resim/curves/two_jet.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/liegroup_concepts.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/inout.hh"

namespace resim::curves {

namespace {
using SE3 = transforms::SE3;
using SO3 = transforms::SO3;

constexpr auto EMPTY_ERR =
    "Cannot query the frame of a curve with no control points";

template <transforms::LieGroupType Group>
const transforms::Frame<Group::DIMS> &reference_frame_impl(
    const std::vector<typename TCurve<Group>::Control> &control_pts) {
  REASSERT(!control_pts.empty(), EMPTY_ERR);
  return control_pts.back().point.frame_from_ref().from();
}

template <transforms::LieGroupType Group>
const transforms::Frame<Group::DIMS> &point_frame_impl(
    const std::vector<typename TCurve<Group>::Control> &control_pts) {
  REASSERT(!control_pts.empty(), EMPTY_ERR);
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
    REASSERT(point.time > control_pts_.back().time, TIME_ERR);
    // Check that both control point frames match.
    constexpr auto FRAME_ERR =
        "Control points must all have the same ref and point frame";
    const bool frame_test = point.point.frame_from_ref().verify_frames(
        this->point_frame(),
        this->reference_frame());
    REASSERT(frame_test, FRAME_ERR);
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

template <transforms::LieGroupType Group>
bool TCurve<Group>::is_framed() const {
  REASSERT(!control_pts_.empty(), EMPTY_ERR);
  return control_pts_.back().point.frame_from_ref().is_framed();
}

template <transforms::LieGroupType Group>
const transforms::Frame<Group::DIMS> &TCurve<Group>::reference_frame() const {
  return reference_frame_impl<Group>(control_pts_);
}

template <transforms::LieGroupType Group>
const transforms::Frame<Group::DIMS> &TCurve<Group>::point_frame() const {
  return point_frame_impl<Group>(control_pts_);
}

template <transforms::LieGroupType Group>
TwoJetL<Group> TCurve<Group>::point_at(const double time) const {
  const PointAtData pd = point_at_impl(time);
  TwoJetL<Group> point = pd.in_segment.curve.point_at(pd.time_nrm);
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
  REASSERT(!segments_.empty(), EMPTY_ERR);
  constexpr auto TIME_RANGE_ERR =
      "Query time must be within the range of times in the control points.";
  const bool time_lo = time < control_pts_.front().time;
  const bool time_hi = time > control_pts_.back().time;
  REASSERT(!time_lo && !time_hi, TIME_RANGE_ERR);
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
  REASSERT(!control_pts_.empty(), EMPTY_ERR);
  return control_pts_.back().time;
}

template <transforms::LieGroupType Group>
double TCurve<Group>::start_time() const {
  REASSERT(!control_pts_.empty(), EMPTY_ERR);
  return control_pts_.front().time;
}

template class TCurve<SE3>;
template class TCurve<SO3>;

}  // namespace resim::curves
