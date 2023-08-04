// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/utils/status_value.hh"
#include "resim/visualization/proto/view_primitive.pb.h"
#include "resim/visualization/view_primitive.hh"

namespace resim::visualization::proto {

void pack(const visualization::ViewPrimitive &in, ViewPrimitive *out);

visualization::ViewPrimitive unpack(const ViewPrimitive &in);

namespace detail {

StatusValue<visualization::ViewPrimitive> unpack(const ViewPrimitive &in);

}

};  // namespace resim::visualization::proto
