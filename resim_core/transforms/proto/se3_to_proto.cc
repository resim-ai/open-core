#include "resim_core/transforms/proto/se3_to_proto.hh"

#include "resim_core/transforms/proto/liegroup_to_proto.hh"
#include "resim_core/transforms/proto/se3.pb.h"
#include "resim_core/transforms/se3.hh"

namespace resim::transforms::proto {

void pack(const transforms::SE3 &in, SE3 *const out) { pack_liegroup(in, out); }

transforms::SE3 unpack(const SE3 &in) {
  return unpack_liegroup<transforms::SE3, SE3>(in);
}

}  // namespace resim::transforms::proto
