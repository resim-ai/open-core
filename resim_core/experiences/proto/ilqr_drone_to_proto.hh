#pragma once

#include "resim_core/experiences/ilqr_drone.hh"
#include "resim_core/experiences/proto/ilqr_drone.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::ILQRDrone &in, ILQRDrone *out);

experiences::ILQRDrone unpack(const ILQRDrone &in);

}  // namespace resim::experiences::proto
