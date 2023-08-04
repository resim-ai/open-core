// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "au/au.hh"
#include "au/units/meters.hh"
#include "au/units/seconds.hh"

namespace resim::dynamics {

// We use the best practices described here:
// https://github.com/aurora-opensource/au/blob/main/docs/howto/new-units.md
using MetersPerSecondSquared =
    decltype(au::Meters{} / au::squared(au::Seconds{}));
constexpr auto meters_per_second_squared = au::meters / au::second / au::second;

constexpr au::QuantityD<MetersPerSecondSquared> GRAVITY_ACCELERATION =
    meters_per_second_squared(9.81);

}  // namespace resim::dynamics
