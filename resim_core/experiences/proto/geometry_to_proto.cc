#include "resim_core/experiences/proto/geometry_to_proto.hh"

#include "resim_core/assert/assert.hh"
#include "resim_core/geometry/proto/wireframe_to_proto.hh"
#include "resim_core/geometry/wireframe.hh"
#include "resim_core/utils/match.hh"
#include "resim_core/utils/proto/uuid_to_proto.hh"

namespace resim::experiences::proto {

void pack(const experiences::Geometry &in, Geometry *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  pack(in.id, out->mutable_id());
  match(in.model, [out](const geometry::Wireframe &wireframe) {
    pack(wireframe, out->mutable_wireframe());
  });
}

experiences::Geometry unpack(const Geometry &in) {
  experiences::Geometry result;
  result.id = unpack(in.id());
  switch (in.model_case()) {
    case Geometry::kWireframe:
      result.model = unpack(in.wireframe());
      break;
    default:
      REASSERT(false, "Unset model in geometry message!");
  }

  return result;
}

}  // namespace resim::experiences::proto
