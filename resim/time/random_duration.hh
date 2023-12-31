// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <random>

#include "resim/time/timestamp.hh"
#include "resim/utils/inout.hh"

namespace resim::time {

// Generate a uniformly random timestamp with the given random number
// generator.
// @param[in] rng - The random number generator to use to generate
//                  results.
template <typename RNG>
Duration random_duration(InOut<RNG> rng) {
  std::uniform_int_distribution dist{
      std::numeric_limits<int64_t>::min(),
      std::numeric_limits<int64_t>::max()};
  return Duration{dist(*rng)};
}

}  // namespace resim::time
