// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/experiences/dynamic_behavior.hh"
#include "resim/experiences/proto/dynamic_behavior.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::DynamicBehavior &in, DynamicBehavior *out);

experiences::DynamicBehavior unpack(const DynamicBehavior &in);

}  // namespace resim::experiences::proto
