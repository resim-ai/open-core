// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/experiences/proto/storyboard.pb.h"
#include "resim/experiences/storyboard.hh"

namespace resim::experiences::proto {

void pack(const experiences::Storyboard &in, Storyboard *out);

experiences::Storyboard unpack(const Storyboard &in);

}  // namespace resim::experiences::proto
