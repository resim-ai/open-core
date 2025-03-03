// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <type_traits>

namespace resim {

// An unsigned integer version of pow() which detects overflow. We
// define 0^0 == 1 for this function which is consistent with IEEE
// (https://en.wikipedia.org/wiki/Zero_to_the_power_of_zero#IEEE_floating-point_standard)
template <typename IntegerType>
IntegerType pow(IntegerType base, IntegerType exponent)
requires std::is_unsigned_v<IntegerType>;

}  // namespace resim
