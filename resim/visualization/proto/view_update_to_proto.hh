// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#pragma once

#include "resim/visualization/proto/view_update.pb.h"
#include "resim/visualization/view_update.hh"

namespace resim::visualization::proto {

void pack(const visualization::ViewUpdate &in, ViewUpdate *out);

visualization::ViewUpdate unpack(const ViewUpdate &in);

};  // namespace resim::visualization::proto
