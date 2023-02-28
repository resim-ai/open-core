#include "resim_core/curves/proto/t_curve_to_proto.hh"

#include <glog/logging.h>

#include "resim_core/assert/assert.hh"
#include "resim_core/curves/proto/t_curve.pb.h"
#include "resim_core/curves/proto/two_jetl_fse3_to_proto.hh"
#include "resim_core/curves/proto/two_jetl_fso3_to_proto.hh"
#include "resim_core/curves/proto/two_jetl_se3_to_proto.hh"
#include "resim_core/curves/proto/two_jetl_so3_to_proto.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::curves::proto {

template <typename Group, typename Msg>
void pack_t_curve(const TCurve<Group> &in, Msg *out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  for (const auto &control : in.control_pts()) {
    double time = control.time;
    typename Msg::Control *proto_control = out->add_points();
    proto_control->set_time(time);
    pack(control.point, proto_control->mutable_point());
  }
}

template void pack_t_curve(const TCurve<transforms::SE3> &, TCurve_SE3 *);
template void pack_t_curve(const TCurve<transforms::SO3> &, TCurve_SO3 *);
template void pack_t_curve(const TCurve<transforms::FSE3> &, TCurve_FSE3 *);
template void pack_t_curve(const TCurve<transforms::FSO3> &, TCurve_FSO3 *);

template <typename Group, typename Msg>
void unpack_t_curve(const Msg &in, InOut<TCurve<Group>> out) {
  *out = TCurve<Group>();
  for (const auto &proto_control : in.points()) {
    TwoJetL<Group> point = unpack(proto_control.point());
    double time = proto_control.time();
    out->append({time, point});
  }
}

template void unpack_t_curve(
    const TCurve_SE3 &,
    InOut<TCurve<transforms::SE3>>);
template void unpack_t_curve(
    const TCurve_SO3 &,
    InOut<TCurve<transforms::SO3>>);
template void unpack_t_curve(
    const TCurve_FSE3 &,
    InOut<TCurve<transforms::FSE3>>);
template void unpack_t_curve(
    const TCurve_FSO3 &,
    InOut<TCurve<transforms::FSO3>>);

}  // namespace resim::curves::proto
