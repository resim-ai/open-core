// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/proto/d_curve_se3_to_proto.hh"

#include "resim/curves/proto/d_curve.pb.h"
#include "resim/curves/proto/d_curve_to_proto.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/inout.hh"

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
