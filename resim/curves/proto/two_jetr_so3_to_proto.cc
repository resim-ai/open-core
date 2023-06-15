#include "resim/curves/proto/two_jetr_so3_to_proto.hh"

#include <glog/logging.h>

#include "resim/curves/proto/two_jet.pb.h"
#include "resim/curves/proto/two_jet_to_proto.hh"
#include "resim/curves/two_jet.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves::proto {

void pack(const TwoJetR<transforms::SO3> &in, TwoJetR_SO3 *const out) {
  pack_two_jetr(in, out);
}

TwoJetR<transforms::SO3> unpack(const TwoJetR_SO3 &in) {
  TwoJetR<transforms::SO3> two_jet;
  unpack_two_jetr(in, InOut(two_jet));
  return two_jet;
}

}  // namespace resim::curves::proto
