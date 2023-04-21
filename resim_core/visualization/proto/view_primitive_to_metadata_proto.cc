#include "resim_core/visualization/proto/view_primitive_to_metadata_proto.hh"

#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/curves/d_curve.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/utils/match.hh"
#include "resim_core/visualization/proto/view_object_metadata.pb.h"
#include "resim_core/visualization/view_primitive.hh"

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
      [out](const transforms::FSE3 &fse3) {
        out->set_type(proto::ReSimType::TYPE_FSE3);
      },
      [out](const transforms::FSO3 &fso3) {
        out->set_type(proto::ReSimType::TYPE_FSO3);
      },
      [out](const curves::DCurve<transforms::SE3> &d_curve_se3) {
        out->set_type(proto::ReSimType::TYPE_DCURVE_SE3);
      },
      [out](const curves::DCurve<transforms::FSE3> &d_curve_fse3) {
        out->set_type(proto::ReSimType::TYPE_DCURVE_FSE3);
      },
      [out](const curves::TCurve<transforms::FSE3> &t_curve) {
        out->set_type(proto::ReSimType::TYPE_TCURVE_FSE3);
      },
      [out](const actor::state::Trajectory &trajectory) {
        out->set_type(proto::ReSimType::TYPE_TRAJECTORY);
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
