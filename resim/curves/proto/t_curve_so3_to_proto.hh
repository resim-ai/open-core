#pragma once

#include "resim/curves/proto/t_curve.pb.h"
#include "resim/curves/t_curve.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves::proto {

void pack(const TCurve<transforms::SO3> &in, TCurve_SO3 *out);

TCurve<transforms::SO3> unpack(const TCurve_SO3 &in);

}  // namespace resim::curves::proto
