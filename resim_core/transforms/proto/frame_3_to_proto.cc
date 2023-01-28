#include "resim_core/transforms/proto/frame_3_to_proto.hh"

#include <glog/logging.h>

#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/proto/frame_3.pb.h"
#include "resim_core/utils/proto/uuid_to_proto.hh"

namespace resim::transforms::proto {

void pack(const transforms::Frame<3> &in, Frame_3 *const out) {
  CHECK(out != nullptr) << "Can't pack into invalid proto!";
  out->Clear();
  pack(in.id(), out->mutable_id());
}

transforms::Frame<3> unpack(const Frame_3 &in) {
  return Frame<3>{unpack(in.id())};
}

};  // namespace resim::transforms::proto
