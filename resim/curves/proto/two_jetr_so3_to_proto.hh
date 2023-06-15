#pragma once

#include "resim/curves/proto/two_jet.pb.h"
#include "resim/curves/two_jet.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves::proto {

void pack(const TwoJetR<transforms::SO3> &in, TwoJetR_SO3 *out);

TwoJetR<transforms::SO3> unpack(const TwoJetR_SO3 &in);

}  // namespace resim::curves::proto
