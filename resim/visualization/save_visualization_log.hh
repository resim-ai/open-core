// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <filesystem>
#include <span>
#include <string_view>

#include "resim/curves/t_curve.hh"
#include "resim/transforms/se3.hh"

namespace resim::visualization {

// Save a visualization log with the given name so we can view the results
// afterwards.
void save_visualization_log(
    const std::span<const curves::TCurve<transforms::SE3>> &t_curves,
    std::string_view filename);

void save_visualization_log(
    const curves::TCurve<transforms::SE3> &t_curve,
    std::string_view filename);

}  // namespace resim::visualization
