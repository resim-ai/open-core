// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/proto/t_curve_se3_to_proto.hh"

#include <glog/logging.h>

#include "resim/curves/proto/t_curve.pb.h"
#include "resim/curves/proto/t_curve_to_proto.hh"
#include "resim/curves/t_curve.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves::proto {

void pack(const TCurve<transforms::SE3> &in, TCurve_SE3 *const out) {
  pack_t_curve(in, out);
}

TCurve<transforms::SE3> unpack(const TCurve_SE3 &in) {
  TCurve<transforms::SE3> t_curve;
  unpack_t_curve(in, InOut(t_curve));
  return t_curve;
}

}  // namespace resim::curves::proto
