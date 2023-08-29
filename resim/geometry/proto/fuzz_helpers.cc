// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/geometry/proto/fuzz_helpers.hh"

namespace resim::geometry::proto {

bool verify_equality(const OrientedBoxSE3 &a, const OrientedBoxSE3 &b) {
  if (not verify_equality(a.reference_from_box(), b.reference_from_box())) {
    return false;
  }

  for (int ii = 0; ii < transforms::SE3::DIMS; ++ii) {
    if (not resim::verify_equality(a.extents(ii), b.extents(ii))) {
      return false;
    }
  }
  return true;
}

}  // namespace resim::geometry::proto
