// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

namespace resim::math {

template <typename Float>
Float clamp(Float x, Float a, Float b) {
  REASSERT(a <= b);
  return x > b ? b : (x < a ? a : x);
}

}  // namespace resim::math
