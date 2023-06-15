#include "resim/visualization/proto/view_primitive_to_metadata_proto.hh"

#include "resim/actor/state/trajectory.hh"
#include "resim/assert/assert.hh"
#include "resim/curves/d_curve.hh"
#include "resim/curves/t_curve.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/framed_vector.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/match.hh"
#include "resim/visualization/proto/view_object_metadata.pb.h"
#include "resim/visualization/view_primitive.hh"

namespace resim::visualization::proto {

void pack_metadata(
    const visualization::ViewPrimitive &in,
    const std::string &name,
    ViewObjectMetadata *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  // we do not use the name from the ViewPrimitive since it is an optional and
  // the name may not have been set by the user, so we need to compute and pass
  // through separately.
  out->set_name(name);
  out->set_file(in.file_name);
  out->set_line_number(in.line_number);
  // Pack the type
  match(
      in.payload,
      [out](const transforms::Frame<3> &frame) {
        out->set_type(proto::ReSimType::TYPE_FRAME);
      },
      [out](const transforms::SE3 &se3) {
        out->set_type(proto::ReSimType::TYPE_SE3);
      },
      [out](const transforms::SO3 &so3) {
        out->set_type(proto::ReSimType::TYPE_SO3);
      },
      [out](const curves::DCurve<transforms::SE3> &d_curve_se3) {
        out->set_type(proto::ReSimType::TYPE_DCURVE_SE3);
      },
      [out](const curves::TCurve<transforms::SE3> &t_curve) {
        out->set_type(proto::ReSimType::TYPE_TCURVE_SE3);
      },
      [out](const actor::state::Trajectory &trajectory) {
        out->set_type(proto::ReSimType::TYPE_TRAJECTORY);
      },
      [out](const transforms::FramedVector<3> &framed_vector) {
        out->set_type(proto::ReSimType::TYPE_FRAMED_VECTOR);
      });
}

void pack_metadata_list(
    const std::vector<ViewObjectMetadata> &in,
    ViewObjectMetadataList *out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  for (const auto &metadata : in) {
    *out->add_metadata() = metadata;
  }
}

};  // namespace resim::visualization::proto
