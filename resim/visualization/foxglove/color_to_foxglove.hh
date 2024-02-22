// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <foxglove/Color.pb.h>

#include "resim/visualization/color.hh"

namespace resim::visualization::foxglove {

// Pack the given color into a ::foxglove::Color message.
void pack_into_foxglove(const Color &in, ::foxglove::Color *out);

}  // namespace resim::visualization::foxglove
