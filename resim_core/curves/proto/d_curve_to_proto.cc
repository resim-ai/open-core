#include "resim_core/curves/proto/d_curve_to_proto.hh"

#include "resim_core/assert/assert.hh"
#include "resim_core/curves/proto/d_curve.pb.h"
#include "resim_core/transforms/proto/se3_to_proto.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::curves::proto {

template <typename Group, typename Msg>
void pack_d_curve(const DCurve<Group> &in, Msg *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();

  for (const auto &control_point : in.control_pts()) {
    pack(*control_point.ref_from_control, out->add_ref_from_control());
  }
}

template <typename Group, typename Msg>
void unpack_d_curve(const Msg &in, InOut<DCurve<Group>> out) {
  std::vector<Group> points;
  points.reserve(in.ref_from_control_size());

  // Iterate through the points
  for (const auto &ref_from_control : in.ref_from_control()) {
    Group group = unpack(ref_from_control);
    points.push_back(group);
  }

  *out = DCurve<Group>(std::move(points));
}

template void pack_d_curve(const DCurve<transforms::SE3> &, DCurve_SE3 *);
template void unpack_d_curve(
    const DCurve_SE3 &,
    InOut<DCurve<transforms::SE3>>);

}  // namespace resim::curves::proto
