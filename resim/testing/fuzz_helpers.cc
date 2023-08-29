// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/testing/fuzz_helpers.hh"

#include <cmath>

#include "resim/math/is_approx.hh"

namespace resim {

bool verify_equality(double a, double b) { return math::is_approx(a, b); }

}  // namespace resim
