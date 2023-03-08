
#include "resim_core/visualization/proto/view_primitive_to_proto.hh"

#include "resim_core/assert/assert.hh"
#include "resim_core/curves/proto/d_curve_fse3_to_proto.hh"
#include "resim_core/curves/proto/d_curve_se3_to_proto.hh"
#include "resim_core/curves/proto/t_curve_fse3_to_proto.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/proto/fse3_to_proto.hh"
#include "resim_core/transforms/proto/se3_to_proto.hh"
#include "resim_core/transforms/proto/so3_to_proto.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/utils/match.hh"
#include "resim_core/utils/proto/uuid_to_proto.hh"

namespace resim::visualization::proto {

void pack(const visualization::ViewPrimitive &in, ViewPrimitive *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  pack(in.id, out->mutable_id());
  match(
      in.payload,
      [out](const transforms::SE3 &se3) { pack(se3, out->mutable_se3()); },
      [out](const transforms::SO3 &so3) { pack(so3, out->mutable_so3()); },
      [out](const transforms::FSE3 &fse3) { pack(fse3, out->mutable_fse3()); },
      [out](const curves::DCurve<transforms::SE3> &d_curve_se3) {
        pack(d_curve_se3, out->mutable_d_curve_se3());
      },
      [out](const curves::DCurve<transforms::FSE3> &d_curve_fse3) {
        pack(d_curve_fse3, out->mutable_d_curve_fse3());
      },
      [out](const curves::TCurve<transforms::FSE3> &t_curve) {
        pack(t_curve, out->mutable_t_curve_fse3());
      });
}

visualization::ViewPrimitive unpack(const ViewPrimitive &in) {
  StatusValue<visualization::ViewPrimitive> result_sv{detail::unpack(in)};
  CHECK_STATUS_OK(result_sv.status());
  return result_sv.value();
}

namespace detail {
StatusValue<visualization::ViewPrimitive> unpack(const ViewPrimitive &in) {
  using StatusValue = StatusValue<visualization::ViewPrimitive>;
  visualization::ViewPrimitive unpacked{
      .id = unpack(in.id()),
  };
  switch (in.payload_case()) {
    case ViewPrimitive::kSe3:
      unpacked.payload = unpack(in.se3());
      break;
    case ViewPrimitive::kSo3:
      unpacked.payload = unpack(in.so3());
      break;
    case ViewPrimitive::kFse3:
      unpacked.payload = unpack(in.fse3());
      break;
    case ViewPrimitive::kDCurveSe3:
      unpacked.payload = unpack(in.d_curve_se3());
      break;
    case ViewPrimitive::kDCurveFse3:
      unpacked.payload = unpack(in.d_curve_fse3());
      break;
    case ViewPrimitive::kTCurveFse3:
      unpacked.payload = unpack(in.t_curve_fse3());
      break;
    default:
      return StatusValue{MAKE_STATUS("Can't unpack unset ViewPrimitive!")};
  }

  return StatusValue{unpacked};
}

}  // namespace detail
};  // namespace resim::visualization::proto
