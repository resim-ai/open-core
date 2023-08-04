// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/experiences/proto/geometry_to_proto.hh"

#include "resim/assert/assert.hh"
#include "resim/geometry/proto/wireframe_to_proto.hh"
#include "resim/geometry/wireframe.hh"
#include "resim/utils/match.hh"
#include "resim/utils/proto/uuid_to_proto.hh"

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
