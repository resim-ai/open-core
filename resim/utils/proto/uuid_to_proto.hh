// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/utils/proto/uuid.pb.h"
#include "resim/utils/uuid.hh"

namespace resim::proto {

void pack(const resim::UUID &in, UUID *out);

resim::UUID unpack(const UUID &in);

};  // namespace resim::proto
