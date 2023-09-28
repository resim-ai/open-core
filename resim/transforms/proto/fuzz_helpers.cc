// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/transforms/proto/fuzz_helpers.hh"

namespace resim::transforms::proto {

bool verify_equality(const SE3 &a, const SE3 &b) {
  return unpack(a).is_approx(unpack(b));
}

}  // namespace resim::transforms::proto
