

#include "resim_core/utils/proto/uuid_to_proto.hh"

#include "resim_core/assert/assert.hh"

namespace resim::proto {

void pack(const resim::UUID &in, UUID *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  out->set_data(in.to_string());
}

resim::UUID unpack(const UUID &in) { return resim::UUID{in.data()}; }

};  // namespace resim::proto
