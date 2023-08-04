// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/geometry/proto/wireframe.pb.h"
#include "resim/geometry/wireframe.hh"

namespace resim::geometry::proto {

void pack(const geometry::Wireframe &in, Wireframe *out);

geometry::Wireframe unpack(const Wireframe &in);

}  // namespace resim::geometry::proto
