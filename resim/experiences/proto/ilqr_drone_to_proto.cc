// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

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
