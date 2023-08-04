// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/proto/view_primitive_to_proto.hh"

#include "resim/actor/state/proto/trajectory_to_proto.hh"
#include "resim/actor/state/trajectory.hh"
#include "resim/assert/assert.hh"
#include "resim/curves/proto/d_curve_se3_to_proto.hh"
#include "resim/curves/proto/t_curve_se3_to_proto.hh"
#include "resim/curves/t_curve.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/framed_vector.hh"
#include "resim/transforms/proto/frame_3_to_proto.hh"
#include "resim/transforms/proto/framed_vector_3_to_proto.hh"
#include "resim/transforms/proto/se3_to_proto.hh"
#include "resim/transforms/proto/so3_to_proto.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/match.hh"
#include "resim/utils/proto/uuid_to_proto.hh"

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
      [out](const curves::DCurve<transforms::SE3> &d_curve) {
        pack(d_curve, out->mutable_d_curve_se3());
      },
      [out](const curves::TCurve<transforms::SE3> &t_curve) {
        pack(t_curve, out->mutable_t_curve_se3());
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
    case ViewPrimitive::kDCurveSe3:
      unpacked.payload = unpack(in.d_curve_se3());
      break;
    case ViewPrimitive::kTCurveSe3:
      unpacked.payload = unpack(in.t_curve_se3());
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
