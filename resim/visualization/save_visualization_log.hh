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
// afterwards. It saves the TEST_UNDECLARED_OUTPUTS_DIR defined by bazel test if
// it's present, otherwise it just saves in the current directory.
// @param[in] t_curves - TCurves to visualize with escalator frames
// @param[in] filename - The name to write the mcap to. Should end with .mcap
void save_visualization_log(
    const std::span<const curves::TCurve<transforms::SE3>> &t_curves,
    std::string_view filename);

// Convenience overload of the above that works with a single t_curve.
// @param[in] t_curves - TCurves to visualize with escalator frames
// @param[in] filename - The name to write the mcap to. Should end with .mcap
void save_visualization_log(
    const curves::TCurve<transforms::SE3> &t_curve,
    std::string_view filename);

}  // namespace resim::visualization
