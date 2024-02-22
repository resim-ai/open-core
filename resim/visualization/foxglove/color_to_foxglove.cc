#include "resim/visualization/foxglove/color_to_foxglove.hh"

#include "resim/assert/assert.hh"

namespace resim::visualization::foxglove {

void pack_into_foxglove(const Color &in, ::foxglove::Color *const out) {
  REASSERT(out != nullptr, "Can't pack invalid color!");
  out->Clear();
  out->set_r(in.r);
  out->set_g(in.g);
  out->set_b(in.b);
  out->set_a(in.a);
}

}  // namespace resim::visualization::foxglove
