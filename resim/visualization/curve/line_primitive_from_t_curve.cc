
#include "resim/visualization/curve/line_primitive_from_t_curve.hh"

#include <cmath>

#include "resim/assert/assert.hh"
#include "resim/time/sample_interval.hh"
#include "resim/visualization/foxglove/color_to_foxglove.hh"
#include "resim/visualization/foxglove/pose_to_foxglove.hh"
#include "resim/visualization/foxglove/vector_to_foxglove.hh"

namespace resim::visualization::curve {

void line_primitive_from_t_curve(
    const curves::TCurve<transforms::SE3> &curve,
    ::foxglove::LinePrimitive *const line,
    const double sample_period,
    const LinePrimitiveOptions &options) {
  REASSERT(line != nullptr, "Can't pack into invalid line primitive!");
  line->Clear();

  line->set_type(::foxglove::LinePrimitive::LINE_STRIP);
  foxglove::pack_into_foxglove(
      transforms::SE3::identity(),
      line->mutable_pose());
  line->set_thickness(options.thickness);
  line->set_scale_invariant(options.scale_invariant);
  foxglove::pack_into_foxglove(options.color, line->mutable_color());

  time::sample_interval(
      curve.start_time(),
      curve.end_time(),
      sample_period,
      [&](const double t) {
        const Eigen::Vector3d point{
            curve.point_at(t).frame_from_ref().inverse().translation()};
        foxglove::pack_into_foxglove(point, line->add_points());
      });
}

}  // namespace resim::visualization::curve
