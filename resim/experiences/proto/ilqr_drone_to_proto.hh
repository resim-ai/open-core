// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/experiences/ilqr_drone.hh"
#include "resim/experiences/proto/ilqr_drone.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::ILQRDrone &in, ILQRDrone *out);

experiences::ILQRDrone unpack(const ILQRDrone &in);

}  // namespace resim::experiences::proto
