
#pragma once

#include "resim/utils/proto/uuid.pb.h"
#include "resim/utils/uuid.hh"

namespace resim::proto {

void pack(const resim::UUID &in, UUID *out);

resim::UUID unpack(const UUID &in);

};  // namespace resim::proto
