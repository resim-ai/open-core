#pragma once

#include <vector>

#include "resim_core/curves/t_curve_segment.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/utils/inout.hh"

namespace resim::curves {
// A time parameterized curve that is built up from segments of quintic Hermite
// curves between TwoJet<Group> control points. The degrees-of-freedom of the
// curve is determined by the underlying LieGroup (e.g. six for SE3). By design
// the curve is C2 smooth.
// Control points appended to the curve must all use the same reference frame.
// If the caller uses a FramedGroup, then this will be checked.
template <transforms::LieGroupType Group>
class TCurve {
 public:
  using Frame = transforms::Frame<Group::DIMS>;
  // Data for a curve control point.
  struct Control {
    // Time of the control point.
    double time;
    // TwoJet specifying he control point.
    TwoJet<Group> point;
  };
  // Data for a curve segment.
  // Segments are between two control points: the origin control point (orig)
  // and the destination control point (dest).
  struct Segment {
    const double orig_time;
    const double dest_time;
    TCurveSegment<Group> curve;
  };
  TCurve() = default;
  // Construct a TCurve from a vector of control points.
  // Note: it is more efficient to add control points directly to the curve
  // using the append method, so use that preferably.
  explicit TCurve(const std::vector<Control> &points);
  // Append a control point to the end of the TCurve.
  void append(Control point);
  // Retrieve the TwoJet for a point at time 'time' along the TCurve
  TwoJet<Group> point_at(double time) const;
  // Overload allowing the caller to pass in their own frame.
  // Note, this method is only valid for FramedGroup<T> types
  TwoJet<Group> point_at(double time, const Frame &point_frame) const;
  // Access a reference to the control points.
  const std::vector<Control> &control_pts() const;
  // Access a reference to the segments.
  const std::vector<Segment> &segments() const;
  // Retrieve the reference frame of the curve's control points.
  // Note, this method is only valid for FramedGroup<T> types
  const Frame &reference_frame() const;

 private:
  // Private data for use in point_at method implementation.
  struct PointAtData {
    // The segment containing the point.
    const Segment &in_segment;
    // The inverse of the time period of the segment.
    const double inv_dt;
    // Normalized time of the point.
    const double time_nrm;
  };
  // Common logic for the point_at methods.
  PointAtData point_at_impl(double time) const;
  void unnormalize_derivatives(double inv_dt, InOut<TwoJet<Group>> point) const;
  // The curve's control points.
  std::vector<Control> control_pts_;
  // The curve's segments.
  std::vector<Segment> segments_;
};

}  // namespace resim::curves
