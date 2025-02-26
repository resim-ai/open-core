#include <utility>

#include "resim/time/proto/time_to_proto.hh"
#include "resim/time/sample_interval.hh"
#include "resim/visualization/curve/esc_frames_from_t_curve.hh"
#include "resim/visualization/curve/line_primitive_from_t_curve.hh"
#include "resim/visualization/curve/poses_in_frame_from_t_curve.hh"
#include "resim/visualization/curve/visualize_t_curve.hh"

namespace resim::visualization::curve {

namespace {

void add_line_primitive_from_t_curve(
    const curves::TCurve<transforms::SE3> &curve,
    const std::string &curve_name,
    const time::Timestamp time,
    const CurveVisualizationOptions &options,
    const LinePrimitiveOptions &line_options,
    InOut<::foxglove::SceneUpdate> update) {
  auto &entity = *update->add_entities();

  resim::time::proto::pack(time, entity.mutable_timestamp());

  const std::string frame_id{curve.reference_frame().id().to_string()};

  entity.set_frame_id(frame_id);
  entity.set_id(curve_name);
  line_primitive_from_t_curve(
      curve,
      entity.add_lines(),
      time::as_seconds(options.line_sample_period),
      line_options);
}

}  // namespace

MultiTCurveVisualizer::MultiTCurveVisualizer(
    CurveVisualizationOptions options,
    std::string scene_update_topic,
    std::string poses_in_frame_topic,
    InOut<McapLogger> logger)
    : options_{options},
      scene_update_topic_{std::move(scene_update_topic)},
      poses_in_frame_topic_{std::move(poses_in_frame_topic)},
      logger_{*logger} {
  logger_.add_proto_channel<::foxglove::SceneUpdate>(scene_update_topic_);
  logger_.add_proto_channel<::foxglove::PosesInFrame>(poses_in_frame_topic_);
}

MultiTCurveVisualizer::~MultiTCurveVisualizer() { write(); }

void MultiTCurveVisualizer::add_curve(
    const curves::TCurve<transforms::SE3> &curve,
    const SingleCurveOptions &curve_options) {
  const time::Timestamp start_time{time::as_duration(curve.start_time())};

  if (not reference_frame_) {
    reference_frame_ = curve.reference_frame();
  } else {
    const bool frames_match =
        reference_frame_.value() == curve.reference_frame();
    REASSERT(
        frames_match,
        "All curves in the set must use the same reference frame");
  }

  add_line_primitive_from_t_curve(
      curve,
      curve_options.unique_name,
      start_time,
      options_,
      curve_options.line_options,
      InOut{scene_update_});

  if (curve_options.include_escalator_frames) {
    esc_frames_from_t_curve(
        curve,
        curve.reference_frame().id().to_string(),
        options_.pose_period,
        options_.publish_rate,
        InOut(poses_in_frame_));
  }
}

void MultiTCurveVisualizer::write() {
  time::Timestamp first_time = time::Timestamp{};
  if (not poses_in_frame_.empty()) {
    first_time = poses_in_frame_.begin()->first;
  }
  logger_.log_proto(scene_update_topic_, first_time, scene_update_);

  for (const auto &[time, poses] : poses_in_frame_) {
    logger_.log_proto(poses_in_frame_topic_, time, poses);
  }
}

void visualize_t_curve(
    const curves::TCurve<transforms::SE3> &curve,
    const CurveVisualizationOptions &options,
    const std::string &scene_update_topic,
    const std::string &poses_in_frame_topic,
    InOut<McapLogger> logger,
    const std::string &curve_name) {
  MultiTCurveVisualizer viz(
      options,
      scene_update_topic,
      poses_in_frame_topic,
      logger);

  viz.add_curve(curve);
  // Note viz writes on destruction.
}

}  // namespace resim::visualization::curve
