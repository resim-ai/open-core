// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <foxglove/Quaternion.pb.h>

#include "resim/transforms/so3.hh"

namespace resim::visualization::foxglove {

// Pack an SO3 into a ::foxglove::Quaternion proto message.
void pack_into_foxglove(const transforms::SO3 &in, ::foxglove::Quaternion *out);

}  // namespace resim::visualization::foxglove
