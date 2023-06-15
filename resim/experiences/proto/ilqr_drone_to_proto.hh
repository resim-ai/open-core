#pragma once

#include "resim/experiences/ilqr_drone.hh"
#include "resim/experiences/proto/ilqr_drone.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::ILQRDrone &in, ILQRDrone *out);

experiences::ILQRDrone unpack(const ILQRDrone &in);

}  // namespace resim::experiences::proto
