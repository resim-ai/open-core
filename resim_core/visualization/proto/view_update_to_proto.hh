
#pragma once

#include "resim_core/visualization/proto/view_update.pb.h"
#include "resim_core/visualization/view_update.hh"

namespace resim::visualization::proto {

void pack(const visualization::ViewUpdate &in, ViewUpdate *out);

visualization::ViewUpdate unpack(const ViewUpdate &in);

};  // namespace resim::visualization::proto
