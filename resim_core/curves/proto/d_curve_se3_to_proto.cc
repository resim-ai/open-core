#include "resim_core/curves/proto/d_curve_se3_to_proto.hh"

#include <glog/logging.h>

#include "resim_core/curves/proto/d_curve.pb.h"
#include "resim_core/curves/proto/d_curve_to_proto.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/inout.hh"

namespace resim::curves::proto {

void pack(const DCurve<transforms::SE3> &in, DCurve_SE3 *const out) {
  pack_d_curve(in, out);
}

DCurve<transforms::SE3> unpack(const DCurve_SE3 &in) {
  DCurve<transforms::SE3> d_curve_se3;
  unpack_d_curve(in, InOut(d_curve_se3));
  return d_curve_se3;
}

}  // namespace resim::curves::proto
