// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <cmath>

#include "resim/geometry/proto/oriented_box.pb.h"
#include "resim/testing/fuzz_helpers.hh"
#include "resim/transforms/proto/fuzz_helpers.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/inout.hh"

namespace resim::geometry::proto {

template <typename Rng>
OrientedBoxSE3 random_element(
    TypeTag<OrientedBoxSE3> /*unused*/,
    InOut<Rng> rng) {
  OrientedBoxSE3 result;
  result.mutable_reference_from_box()->CopyFrom(
      random_element<transforms::proto::SE3>(rng));

  for (int ii = 0; ii < transforms::SE3::DIMS; ++ii) {
    result.add_extents(std::fabs(random_element<double>(rng)));
  }
  return result;
}

bool verify_equality(const OrientedBoxSE3 &a, const OrientedBoxSE3 &b);

}  // namespace resim::geometry::proto
