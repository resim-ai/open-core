#pragma once

#include "resim_core/curves/proto/two_jet.pb.h"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::curves::proto {

void pack(const TwoJetL<transforms::SE3> &in, TwoJetL_SE3 *out);

TwoJetL<transforms::SE3> unpack(const TwoJetL_SE3 &in);

}  // namespace resim::curves::proto
