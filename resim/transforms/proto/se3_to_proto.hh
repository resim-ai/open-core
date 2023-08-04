// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/transforms/proto/se3.pb.h"
#include "resim/transforms/se3.hh"

namespace resim::transforms::proto {

void pack(const transforms::SE3 &in, SE3 *out);

transforms::SE3 unpack(const SE3 &in);

}  // namespace resim::transforms::proto
