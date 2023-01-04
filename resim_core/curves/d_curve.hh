#pragma once

#include <initializer_list>
#include <memory>
#include <vector>

#include "resim_core/transforms/frame.hh"

namespace resim::curves {

// An arc length parameterized curve that is built up from segments of LieGroup
// transforms. Segments are geodesic curves with the same number of degrees of
// freedom as the underlying LieGroup (e.g. six for SE3). By design the curve is
// C1 smooth. Control points appended to the curve must all use the same
// reference frame. If the caller uses a FramedGroup, then this will be checked.
template <typename Group>
class DCurve {
 public:
  // Data for a curve control point.
  struct Control {
    // Position of the control point in arc length along the curve.
    const double arc_length;
    // Transform from the frame of the control point to the reference frame.
    std::shared_ptr<Group> ref_from_control;
  };
  // Data for a curve segment. Note some data are shared with control points.
  // Segments are between two control points: the origin control point (orig)
  // and the destination control point (dest).
  struct Segment {
    const double orig_arc_length;
    const double dest_arc_length;
    std::shared_ptr<Group> ref_from_orig;
    std::shared_ptr<Group> ref_from_dest;
    // Transform from the destination control point frame to the origin.
    const Group orig_from_dest;
  };
  DCurve() = default;
  // Construct a DCurve from a vector of control points.
  explicit DCurve(const std::vector<Group> &points);
  // Support braces initialization from control points
  // e.g. DCurve<SE3> curve{p0, p1, p2};
  DCurve(std::initializer_list<Group> points);
  // Append a control point to the end of the DCurve.
  void append(Group ref_from_control);
  // Support appending a brace enclosed list.
  // e.g. curve.append({p0, p1, p2});
  void append(std::initializer_list<Group> points);
  // Retrieve the transform from a frame at a point arc_length along the DCurve
  // to the reference frame.
  Group point_at(double arc_length) const;
  // Retrieve the transform from a frame at a point arc_length along the DCurve
  // to the reference frame. The frame of the point will be the user provided
  // frame 'from'. Note this method is only valid for FramedGroup<T> types.
  Group point_at(double arc_length, const transforms::Frame<Group::DIMS> &from)
      const;
  // Access a reference to the control points.
  const std::vector<Control> &control_pts() const;
  // Access a reference to the segments.
  const std::vector<Segment> &segments() const;
  // Retrieve the total length of the curve.
  double curve_length() const;
  // Retrieve the reference frame of the curve's control points.
  // Note, this method is only valid for FramedGroup<T> types
  const transforms::Frame<Group::DIMS> &reference_frame() const;

 private:
  // Private data for use in point_at method implementation.
  struct PointAtData {
    // Dimensionless position of the point along the Segment [0,1].
    const double fraction;
    // Pointer to the Segment containing the point.
    const typename std::vector<Segment>::const_iterator s;
  };
  // Common logic for the point_at methods.
  PointAtData point_at_impl(double arc_length) const;
  // The curve's control points.
  std::vector<Control> control_pts_;
  // The curve's segments.
  std::vector<Segment> segments_;
};

}  // namespace resim::curves
