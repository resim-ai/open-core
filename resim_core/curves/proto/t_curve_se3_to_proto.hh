#pragma once

#include "resim_core/curves/proto/t_curve.pb.h"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::curves::proto {

void pack(const TCurve<transforms::SE3> &in, TCurve_SE3 *out);

TCurve<transforms::SE3> unpack(const TCurve_SE3 &in);

}  // namespace resim::curves::proto
