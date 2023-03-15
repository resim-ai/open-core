#include "resim_core/curves/d_curve.hh"

#include "resim_core/assert/assert.hh"
#include "resim_core/transforms/framed_group.hh"

namespace resim::curves {

namespace {
using SE3 = transforms::SE3;
using FSE3 = transforms::FSE3;
}  // namespace

template <typename Group>
DCurve<Group>::DCurve(const std::vector<Group> &points) {
  for (const auto &point : points) {
    append(point);
  }
}

template <typename Group>
DCurve<Group>::DCurve(std::initializer_list<Group> points) {
  append(points);
}

template <typename Group>
void DCurve<Group>::append(Group ref_from_control) {
  // Note we don't need to check reference frames explicitly because the
  // composition below (when computing arc length) will fail if the frames are
  // inconsistent.
  constexpr double ZERO_LENGTH_M = 0;
  auto ref_from_control_ptr =
      std::make_shared<Group>(std::move(ref_from_control));
  if (control_pts_.empty()) {
    control_pts_.push_back(
        {.arc_length = ZERO_LENGTH_M,
         .ref_from_control = ref_from_control_ptr});
  } else {
    // Build a segment;
    const auto ref_from_orig = control_pts_.back().ref_from_control;
    const Group orig_from_control =
        ref_from_orig->inverse() * *ref_from_control_ptr;
    const double segment_length = orig_from_control.arc_length();
    const double orig_arc_length = control_pts_.back().arc_length;
    control_pts_.push_back(
        {.arc_length = control_pts_.back().arc_length + segment_length,
         .ref_from_control = ref_from_control_ptr});
    const double dest_arc_length = control_pts_.back().arc_length;
    segments_.push_back(
        {.orig_arc_length = orig_arc_length,
         .dest_arc_length = dest_arc_length,
         .ref_from_orig = ref_from_orig,
         .ref_from_dest = control_pts_.back().ref_from_control,
         .orig_from_dest = std::move(orig_from_control)});
  }
}

template <typename Group>
void DCurve<Group>::append(std::initializer_list<Group> points) {
  for (const auto &point : points) {
    append(point);
  }
}

template <typename Group>
Group DCurve<Group>::point_at(double arc_length) const {
  const PointAtData data = point_at_impl(arc_length);
  const Group orig_from_query_point =
      data.s->orig_from_dest.interp(data.fraction);
  return *data.s->ref_from_orig * orig_from_query_point;
}

template <>
FSE3 DCurve<FSE3>::point_at(
    double arc_length,
    const transforms::Frame<FSE3::DIMS> &from) const {
  const PointAtData data = point_at_impl(arc_length);
  const FSE3 orig_from_query_point =
      data.s->orig_from_dest.interp(data.fraction, from);
  return *data.s->ref_from_orig * orig_from_query_point;
}

template <typename Group>
const std::vector<typename DCurve<Group>::Control> &DCurve<Group>::control_pts()
    const {
  return control_pts_;
}

template <typename Group>
const std::vector<typename DCurve<Group>::Segment> &DCurve<Group>::segments()
    const {
  return segments_;
}

template <typename Group>
double DCurve<Group>::curve_length() const {
  REASSERT(!control_pts_.empty(), "Attempt to access length of empty curve.");
  return control_pts_.back().arc_length;
}

template <>
const transforms::Frame<FSE3::DIMS> &DCurve<FSE3>::reference_frame() const {
  return control_pts_.front().ref_from_control->into();
}

template <typename Group>
typename DCurve<Group>::PointAtData DCurve<Group>::point_at_impl(
    double arc_length) const {
  constexpr double ZERO = 0;
  REASSERT(arc_length >= ZERO, "Arc length values must be positive.");
  constexpr auto LENGTH_ERROR_MSG =
      "Attempt to query a point at an arc length longer that the curve.";
  REASSERT(arc_length <= curve_length(), LENGTH_ERROR_MSG);
  const auto s = std::find_if(
      segments_.begin(),
      segments_.end(),
      [&arc_length](const Segment &segment) {
        return (arc_length >= segment.orig_arc_length);
      });

  REASSERT(s != segments_.end(), "Did not find valid segment.");

  const double fraction = (arc_length - s->orig_arc_length) /
                          (s->dest_arc_length - s->orig_arc_length);
  return {.fraction = fraction, .s = s};
}

template class DCurve<SE3>;
template class DCurve<FSE3>;

}  // namespace resim::curves
