// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/experiences/location_condition.hh"
#include "resim/experiences/proto/location_condition.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::LocationCondition &in, LocationCondition *out);

experiences::LocationCondition unpack(const LocationCondition &in);

}  // namespace resim::experiences::proto
