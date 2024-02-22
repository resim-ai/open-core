// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/foxglove/wireframe_to_foxglove.hh"

#include "resim/assert/assert.hh"
#include "resim/transforms/se3.hh"
#include "resim/visualization/color.hh"
#include "resim/visualization/foxglove/color_to_foxglove.hh"
#include "resim/visualization/foxglove/pose_to_foxglove.hh"
#include "resim/visualization/foxglove/vector_to_foxglove.hh"

namespace resim::visualization::foxglove {
namespace {
constexpr double DEFAULT_THICKNESS = 2;
constexpr bool DEFAULT_SCALE_INVARIANT = true;
}  // namespace

void pack_into_foxglove(
    const geometry::Wireframe &in,
    ::foxglove::LinePrimitive *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid line primitive!");
  out->Clear();
  out->set_type(::foxglove::LinePrimitive::LINE_LIST);
  pack_into_foxglove(transforms::SE3::identity(), out->mutable_pose());
  out->set_thickness(DEFAULT_THICKNESS);
  out->set_scale_invariant(DEFAULT_SCALE_INVARIANT);
  pack_into_foxglove(colors::CHARTREUSE, out->mutable_color());

  for (const auto &[p1, p2] : in.edges()) {
    pack_into_foxglove(in.points().at(p1), out->add_points());
    pack_into_foxglove(in.points().at(p2), out->add_points());
  }
}

}  // namespace resim::visualization::foxglove
