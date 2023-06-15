#include "resim/transforms/proto/so3_to_proto.hh"

#include "resim/transforms/proto/liegroup_to_proto.hh"
#include "resim/transforms/proto/so3.pb.h"
#include "resim/transforms/so3.hh"

namespace resim::transforms::proto {

void pack(const transforms::SO3 &in, SO3 *const out) { pack_liegroup(in, out); }

transforms::SO3 unpack(const SO3 &in) {
  transforms::SO3 transform;
  unpack_liegroup(in, InOut(transform));
  return transform;
}

}  // namespace resim::transforms::proto
