
#pragma once

#include "resim_core/utils/proto/uuid.pb.h"
#include "resim_core/utils/uuid.hh"

namespace resim::proto {

void pack(const resim::UUID &in, UUID *out);

resim::UUID unpack(const UUID &in);

};  // namespace resim::proto
