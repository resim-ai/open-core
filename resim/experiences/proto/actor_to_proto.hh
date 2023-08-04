// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/experiences/actor.hh"
#include "resim/experiences/proto/actor.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::Actor &in, Actor *out);

experiences::Actor unpack(const Actor &in);

}  // namespace resim::experiences::proto
