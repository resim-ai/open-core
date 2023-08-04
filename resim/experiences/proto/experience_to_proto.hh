// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#pragma once

#include "resim/experiences/experience.hh"
#include "resim/experiences/proto/experience.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::Experience &in, Experience *out);

experiences::Experience unpack(const Experience &in);

}  // namespace resim::experiences::proto
