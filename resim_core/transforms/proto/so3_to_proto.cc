#include "resim_core/transforms/proto/so3_to_proto.hh"

#include "resim_core/transforms/proto/liegroup_to_proto.hh"
#include "resim_core/transforms/proto/so3.pb.h"
#include "resim_core/transforms/so3.hh"

namespace resim::transforms::proto {

void pack(const transforms::SO3 &in, SO3 *const out) { pack_liegroup(in, out); }

transforms::SO3 unpack(const SO3 &in) {
  return unpack_liegroup<transforms::SO3, SO3>(in);
}

}  // namespace resim::transforms::proto
