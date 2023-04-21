
#include "resim_core/visualization/proto/view_primitive_to_proto.hh"

#include "resim_core/actor/state/proto/trajectory_to_proto.hh"
#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/curves/proto/d_curve_fse3_to_proto.hh"
#include "resim_core/curves/proto/d_curve_se3_to_proto.hh"
#include "resim_core/curves/proto/t_curve_fse3_to_proto.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_vector.hh"
#include "resim_core/transforms/proto/frame_3_to_proto.hh"
#include "resim_core/transforms/proto/framed_vector_3_to_proto.hh"
#include "resim_core/transforms/proto/fse3_to_proto.hh"
#include "resim_core/transforms/proto/fso3_to_proto.hh"
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
  // Pack user defined name if it exists:
  if (in.user_defined_name.has_value()) {
    out->set_user_defined_name(in.user_defined_name.value());
  }
  // Pack file name and line number:
  out->set_file_name(in.file_name);
  out->set_line_number(in.line_number);
  // Pack the payload:
  match(
      in.payload,
      [out](const transforms::Frame<3> &frame) {
        pack(frame, out->mutable_frame());
      },
      [out](const transforms::SE3 &se3) { pack(se3, out->mutable_se3()); },
      [out](const transforms::SO3 &so3) { pack(so3, out->mutable_so3()); },
      [out](const transforms::FSE3 &fse3) { pack(fse3, out->mutable_fse3()); },
      [out](const transforms::FSO3 &fso3) { pack(fso3, out->mutable_fso3()); },
      [out](const curves::DCurve<transforms::SE3> &d_curve_se3) {
        pack(d_curve_se3, out->mutable_d_curve_se3());
      },
      [out](const curves::DCurve<transforms::FSE3> &d_curve_fse3) {
        pack(d_curve_fse3, out->mutable_d_curve_fse3());
      },
      [out](const curves::TCurve<transforms::FSE3> &t_curve) {
        pack(t_curve, out->mutable_t_curve_fse3());
      },
      [out](const actor::state::Trajectory &trajectory) {
        pack(trajectory, out->mutable_trajectory());
      },
      [out](const transforms::FramedVector<3> &framed_vector) {
        pack(framed_vector, out->mutable_framed_vector());
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
  // Unpack user defined name if it exists:
  if (!in.user_defined_name().empty()) {
    unpacked.user_defined_name = in.user_defined_name();
  }
  // Unpack file name and line number:
  unpacked.file_name = in.file_name();
  unpacked.line_number = in.line_number();
  // Unpack the payload:
  switch (in.payload_case()) {
    case ViewPrimitive::kFrame:
      unpacked.payload = unpack(in.frame());
      break;
    case ViewPrimitive::kSe3:
      unpacked.payload = unpack(in.se3());
      break;
    case ViewPrimitive::kSo3:
      unpacked.payload = unpack(in.so3());
      break;
    case ViewPrimitive::kFse3:
      unpacked.payload = unpack(in.fse3());
      break;
    case ViewPrimitive::kFso3:
      unpacked.payload = unpack(in.fso3());
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
    case ViewPrimitive::kTrajectory:
      unpacked.payload = unpack(in.trajectory());
      break;
    case ViewPrimitive::kFramedVector:
      unpacked.payload = unpack(in.framed_vector());
      break;
    default:
      return StatusValue{MAKE_STATUS("Can't unpack unset ViewPrimitive!")};
  }

  return StatusValue{unpacked};
}

}  // namespace detail
};  // namespace resim::visualization::proto
