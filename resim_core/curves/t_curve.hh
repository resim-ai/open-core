#pragma once

#include <initializer_list>
#include <vector>

#include "resim_core/curves/t_curve_segment.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/utils/inout.hh"

namespace resim::curves {
// A time parameterized curve that is built up from segments of quintic Hermite
// curves between TwoJetL<Group> control points. The degrees-of-freedom of the
// curve is determined by the underlying LieGroup (e.g. six for SE3). By design
// the curve is C2 smooth.
//
// Control points appended to the curve must all use the same reference frame
// *and* the same point frame (which is required in downstream uses). If the
// group elements are framed, frames are checked for consistency of frames.
//
// This design decision, requiring the same point frame, can be revisited should
// a use case for a TCurve with differing point frames emerge.
template <transforms::LieGroupType Group>
class TCurve {
 public:
  using Frame = transforms::Frame<Group::DIMS>;
  // Data for a curve control point.
  struct Control {
    // Time of the control point.
    double time;
    // TwoJet specifying the control point.
    TwoJetL<Group> point;
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
  // Support braces initialization from control points
  // e.g. TCurve<SE3> curve{p0, p1, p2};
  TCurve(std::initializer_list<Control> points);
  // Append a control point to the end of the TCurve.
  void append(Control point);
  // Support appending a brace enclosed list.
  // e.g. curve.append({p0, p1, p2});
  void append(std::initializer_list<Control> points);
  // Retrieve the TwoJet for a point at time 'time' along the TCurve
  TwoJetL<Group> point_at(double time) const;
  // Access a reference to the control points.
  const std::vector<Control> &control_pts() const;
  // Access a reference to the segments.
  const std::vector<Segment> &segments() const;
  // Returns whether this curve is built from liegroup objects with frames.
  bool is_framed() const;
  // Retrieve the reference frame of the curve's control points.
  const Frame &reference_frame() const;
  // Retrieve the point frame of the curve's control points.
  const Frame &point_frame() const;
  // Getters for the start and end time
  double start_time() const;
  double end_time() const;

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
  void unnormalize_derivatives(double inv_dt, InOut<TwoJetL<Group>> point)
      const;
  // The curve's control points.
  std::vector<Control> control_pts_;
  // The curve's segments.
  std::vector<Segment> segments_;
};

}  // namespace resim::curves
