// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/curves/proto/two_jet.pb.h"
#include "resim/curves/two_jet.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves::proto {

void pack(const TwoJetR<transforms::SO3> &in, TwoJetR_SO3 *out);

TwoJetR<transforms::SO3> unpack(const TwoJetR_SO3 &in);

}  // namespace resim::curves::proto
