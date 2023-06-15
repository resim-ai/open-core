#include "resim/experiences/proto/ilqr_drone_to_proto.hh"

#include "resim/assert/assert.hh"
#include "resim/experiences/ilqr_drone.hh"
#include "resim/experiences/proto/ilqr_drone.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::ILQRDrone &in, ILQRDrone *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
}

experiences::ILQRDrone unpack(const ILQRDrone &in) {
  experiences::ILQRDrone result;
  return result;
}

}  // namespace resim::experiences::proto
