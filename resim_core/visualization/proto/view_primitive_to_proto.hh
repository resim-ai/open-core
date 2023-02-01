
#pragma once

#include "resim_core/utils/status_value.hh"
#include "resim_core/visualization/proto/view_primitive.pb.h"
#include "resim_core/visualization/view_primitive.hh"

namespace resim::visualization::proto {

void pack(const visualization::ViewPrimitive &in, ViewPrimitive *out);

visualization::ViewPrimitive unpack(const ViewPrimitive &in);

namespace detail {

StatusValue<visualization::ViewPrimitive> unpack(const ViewPrimitive &in);

}

};  // namespace resim::visualization::proto
