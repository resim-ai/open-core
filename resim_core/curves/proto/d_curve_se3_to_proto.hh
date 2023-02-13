#pragma once

#include "resim_core/curves/d_curve.hh"
#include "resim_core/curves/proto/d_curve.pb.h"
#include "resim_core/transforms/se3.hh"

namespace resim::curves::proto {

void pack(const DCurve<transforms::SE3> &in, DCurve_SE3 *out);

DCurve<transforms::SE3> unpack(const DCurve_SE3 &in);

}  // namespace resim::curves::proto
