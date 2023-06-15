#pragma once

#include "resim/curves/proto/two_jet.pb.h"
#include "resim/curves/two_jet.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves::proto {

void pack(const TwoJetR<transforms::SE3> &in, TwoJetR_SE3 *out);

TwoJetR<transforms::SE3> unpack(const TwoJetR_SE3 &in);

}  // namespace resim::curves::proto
