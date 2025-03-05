// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <foxglove/PosesInFrame.pb.h>
#include <foxglove/SceneUpdate.pb.h>

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "resim/curves/t_curve.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/mcap_logger.hh"
#include "resim/visualization/curve/line_primitive_options.hh"

namespace resim::visualization::curve {

// Options for the visualization of sets of TCurves.
struct CurveVisualizationOptions {
  static constexpr int DEFAULT_PUBLISH_RATE_MS = 50;
  static constexpr int DEFAULT_LINE_SAMPLE_PERIOD_MS = 100;
  static constexpr int DEFAULT_POSE_PERIOD_S = 1;

  // Publish rate for the line and escalator frame messages
  time::Duration publish_rate{
      std::chrono::milliseconds(DEFAULT_PUBLISH_RATE_MS)};

  // How finely we sample the poly-line for the line visualization of the
  // trajectory.
  time::Duration line_sample_period{
      std::chrono::milliseconds(DEFAULT_LINE_SAMPLE_PERIOD_MS)};

  // The time interval along the curve between adjacent escalator frames.
  time::Duration pose_period{std::chrono::seconds(DEFAULT_POSE_PERIOD_S)};
};

// Specific per-curve options.
struct SingleCurveOptions {
  // Its important that the curve's name is unique so we initialize it
  // with a uuid.
  std::string unique_name = UUID::new_uuid().to_string();
  // Options for the line element of the visualization.
  LinePrimitiveOptions line_options{};
  // Choose whether to display escalator frames.
  bool include_escalator_frames = true;
};

// This class logs SceneUpdates containing line visualizations of a given set of
// TCurves. Also PosesInFrame to visualize how the orientations vary over the
// curves. As time progresses, the PosesInFrame progress along the curve giving
// an "escalator"-like appearance to visualize the derivatives of the
// orientation as time progresses.
class MultiTCurveVisualizer {
 public:
  // Provide the required options, topic, and logger parameters.
  // @param[in] options - The options described above.
  // @param[in] scene_update_topic - The topic to log line SceneUpdates to.
  // @param[in] poses_in_frame_topic - The topic to log escalator frame
  //                                   PosesInFrames to.
  // @param[inout] logger - The logger to log the visualization messages to.
  MultiTCurveVisualizer(
      CurveVisualizationOptions options,
      std::string scene_update_topic,
      std::string poses_in_frame_topic,
      InOut<McapLogger> logger);

  ~MultiTCurveVisualizer();
  // Non default destructor means we should delete copy/move assignment
  // operators. Copy/move construction is fine.
  MultiTCurveVisualizer(const MultiTCurveVisualizer &) = default;
  MultiTCurveVisualizer(MultiTCurveVisualizer &&) = default;
  MultiTCurveVisualizer &operator=(const MultiTCurveVisualizer &) = delete;
  MultiTCurveVisualizer &operator=(MultiTCurveVisualizer &&) = delete;

  // @param[in] curve - The curve to visualize.
  // @param[in] line_options - Specific option for the line primitive.
  // @param[optional] curve_name - A unique name for the curve.
  // @param[optional] include_escalator_frames - visualize these, or not.
  void add_curve(
      const resim::curves::TCurve<transforms::SE3> &curve,
      const SingleCurveOptions &curve_options = SingleCurveOptions{});

 private:
  void write();

  CurveVisualizationOptions options_;
  std::string scene_update_topic_;
  std::string poses_in_frame_topic_;
  McapLogger *logger_;

  ::foxglove::SceneUpdate scene_update_;
  std::map<time::Timestamp, ::foxglove::PosesInFrame> poses_in_frame_;

  std::optional<transforms::Frame<transforms::SE3::DIMS>> reference_frame_;
};

// This convenience function wraps MultiTCurveVisualize, providing an easy
// solution for the use who wants to visualize a single curve.
// @param[in] curve - The curve to visualize.
// @param[in] options - The options described above.
// @param[in] scene_update_topic - The topic to log line SceneUpdates to.
// @param[in] poses_in_frame_topic - The topic to log escalator frame
//                                   PosesInFrames to.
// @param[inout] logger - The logger to log the visualization messages to.
// @param[optional] curve_name - A unique name for the curve.
void visualize_t_curve(
    const curves::TCurve<transforms::SE3> &curve,
    const CurveVisualizationOptions &options,
    const std::string &scene_update_topic,
    const std::string &poses_in_frame_topic,
    InOut<McapLogger> logger,
    const std::string &curve_name = UUID::new_uuid().to_string());

}  // namespace resim::visualization::curve
