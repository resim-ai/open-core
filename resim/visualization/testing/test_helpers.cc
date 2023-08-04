// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/testing/test_helpers.hh"

#include "resim/transforms/se3.hh"
#include "resim/utils/match.hh"

namespace resim::visualization::testing {

using transforms::SE3;
using TangentVector = SE3::TangentVector;

bool primitives_equal(const ViewPrimitive &a, const ViewPrimitive &b) {
  if (a.id != b.id) {
    return false;
  }
  return std::get<SE3>(a.payload).is_approx(std::get<SE3>(b.payload));
}

}  // namespace resim::visualization::testing
