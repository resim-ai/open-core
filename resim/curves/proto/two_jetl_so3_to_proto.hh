#pragma once

#include "resim/curves/proto/two_jet.pb.h"
#include "resim/curves/two_jet.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves::proto {

void pack(const TwoJetL<transforms::SO3> &in, TwoJetL_SO3 *out);

TwoJetL<transforms::SO3> unpack(const TwoJetL_SO3 &in);

}  // namespace resim::curves::proto
