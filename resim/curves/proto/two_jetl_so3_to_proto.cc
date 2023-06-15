#include "resim/curves/proto/two_jetl_so3_to_proto.hh"

#include "resim/assert/assert.hh"
#include "resim/curves/proto/two_jet.pb.h"
#include "resim/curves/proto/two_jet_to_proto.hh"
#include "resim/curves/two_jet.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves::proto {

void pack(const TwoJetL<transforms::SO3> &in, TwoJetL_SO3 *const out) {
  pack_two_jetl(in, out);
}

TwoJetL<transforms::SO3> unpack(const TwoJetL_SO3 &in) {
  TwoJetL<transforms::SO3> two_jet;
  unpack_two_jetl(in, InOut(two_jet));
  return two_jet;
}

}  // namespace resim::curves::proto
