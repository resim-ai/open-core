#include "resim_core/curves/proto/two_jetl_se3_to_proto.hh"

#include <glog/logging.h>

#include "resim_core/curves/proto/two_jet.pb.h"
#include "resim_core/curves/proto/two_jet_to_proto.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::curves::proto {

void pack(const TwoJetL<transforms::SE3> &in, TwoJetL_SE3 *const out) {
  pack_two_jet(in, out);
}

TwoJetL<transforms::SE3> unpack(const TwoJetL_SE3 &in) {
  TwoJetL<transforms::SE3> two_jet;
  unpack_two_jet(in, InOut(two_jet));
  return two_jet;
}

}  // namespace resim::curves::proto
