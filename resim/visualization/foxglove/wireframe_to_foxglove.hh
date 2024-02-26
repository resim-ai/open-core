// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <foxglove/LinePrimitive.pb.h>

#include "resim/geometry/wireframe.hh"

namespace resim::visualization::foxglove {

// Pack a Wireframe into a LinePrimitive of the LINE_LIST type. Defaults to a
// scale invariant thickness of 2 and a CHARTREUSE color.
// TODO(michael) Make color configurable
void pack_into_foxglove(
    const geometry::Wireframe &in,
    ::foxglove::LinePrimitive *out);

}  // namespace resim::visualization::foxglove
