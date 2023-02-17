#include "resim_core/curves/proto/t_curve_so3_to_proto.hh"

#include <glog/logging.h>

#include "resim_core/curves/proto/t_curve.pb.h"
#include "resim_core/curves/proto/t_curve_to_proto.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::curves::proto {

void pack(const TCurve<transforms::SO3> &in, TCurve_SO3 *const out) {
  pack_t_curve(in, out);
}

TCurve<transforms::SO3> unpack(const TCurve_SO3 &in) {
  TCurve<transforms::SO3> t_curve;
  unpack_t_curve(in, InOut(t_curve));
  return t_curve;
}

}  // namespace resim::curves::proto
