// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/experiences/geometry.hh"
#include "resim/experiences/proto/geometry.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::Geometry &in, Geometry *out);

experiences::Geometry unpack(const Geometry &in);

}  // namespace resim::experiences::proto
