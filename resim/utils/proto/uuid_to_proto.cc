// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/utils/proto/uuid_to_proto.hh"

#include "resim/assert/assert.hh"

namespace resim::proto {

void pack(const resim::UUID &in, UUID *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  out->set_data(in.to_string());
}

resim::UUID unpack(const UUID &in) { return resim::UUID{in.data()}; }

};  // namespace resim::proto
