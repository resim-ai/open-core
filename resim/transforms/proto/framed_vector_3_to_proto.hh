#pragma once

#include "resim/transforms/framed_vector.hh"
#include "resim/transforms/proto/framed_vector_3.pb.h"

namespace resim::transforms::proto {

void pack(const transforms::FramedVector<3> &in, FramedVector_3 *out);

transforms::FramedVector<3> unpack(const FramedVector_3 &in);

}  // namespace resim::transforms::proto
