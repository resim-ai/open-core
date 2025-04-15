// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/t_curve.hh"
#include "resim/utils/mcap_logger.hh"
#include "resim/visualization/curve/visualize_t_curve.hh"
#include "resim/visualization/save_visualization_log.hh"

namespace resim::visualization {

void save_visualization_log(
    const std::span<const curves::TCurve<transforms::SE3>> &t_curves,
    const std::string_view filename) {
  // TODO(mbauer) Set maybe_path_outputs from TEST_UNDECLARED_OUTPUTS_DIR"
  // again.
  const char *maybe_outputs_dir = nullptr;
  const std::filesystem::path OUTPUTS_DIR{
      maybe_outputs_dir != nullptr ? maybe_outputs_dir : "."};
  resim::McapLogger logger{OUTPUTS_DIR / filename};

  curve::MultiTCurveVisualizer visualizer{
      curve::CurveVisualizationOptions(),
      "/update",
      "/poses",
      InOut(logger)};

  for (const auto &t_curve : t_curves) {
    visualizer.add_curve(t_curve);
  }
}

void save_visualization_log(
    const curves::TCurve<transforms::SE3> &t_curve,
    std::string_view filename) {
  save_visualization_log(
      std::vector<curves::TCurve<transforms::SE3>>{t_curve},
      filename);
}

}  // namespace resim::visualization
