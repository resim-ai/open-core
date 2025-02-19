// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/optimization/pose_error_model.hh"

#include <fmt/core.h>
#include <foxglove/LinePrimitive.pb.h>
#include <foxglove/PosesInFrame.pb.h>
#include <foxglove/SceneUpdate.pb.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <random>
#include <utility>
#include <vector>

#include "resim/assert/assert.hh"
#include "resim/curves/optimization/control_point_parameter.hh"
#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/mcap_logger.hh"
#include "resim/visualization/color.hh"
#include "resim/visualization/foxglove/color_to_foxglove.hh"
#include "resim/visualization/foxglove/pose_to_foxglove.hh"
#include "resim/visualization/foxglove/vector_to_foxglove.hh"
namespace resim::curves::optimization {

using transforms::SE3;
using transforms::SO3;
using Vec3 = Eigen::Vector3d;
using std::literals::chrono_literals::operator""ms;

TEST(PoseErrorModelTest, TestConstructor) {
  // SETUP
  const math::GaussNewtonOptimizer optimizer{
      math::GaussNewtonOptimizer::Options{}};
  const std::string key = "trajectory";

  constexpr double SECOND_POINT_OFFSET = -2.0;
  std::vector<TimedPose> poses{
      TimedPose{.time = 0.0, .observation_from_scene = SE3::identity()},
      TimedPose{
          .time = 1.0,
          .observation_from_scene = SE3{SECOND_POINT_OFFSET * Vec3::UnitX()}},
  };
  const std::size_t num_poses = poses.size();

  // ACTION
  const PoseErrorModel error_model{optimizer, key, std::move(poses)};

  // VERIFICATION
  EXPECT_EQ(error_model.dof(), num_poses * SE3::DOF);
}

TEST(PoseErrorModelTest, TestOptimize) {
  // SETUP
  math::GaussNewtonOptimizer optimizer{math::GaussNewtonOptimizer::Options{}};
  const std::string key = "trajectory";
  constexpr double SECOND_POINT_OFFSET = -2.0;

  const std::vector<TimedPose> poses{
      TimedPose{.time = 2.5, .observation_from_scene = SE3::identity()},
      TimedPose{
          .time = 7.5,
          .observation_from_scene = SE3{SECOND_POINT_OFFSET * Vec3::UnitX()}},
  };

  std::unique_ptr<math::ErrorModel> error_model =
      std::make_unique<PoseErrorModel>(optimizer, key, poses);

  constexpr double START_TIME = 0.0;
  constexpr double END_TIME = 10.0;
  std::vector<ControlPointParameter> parameters = {
      ControlPointParameter{
          .value =
              TCurve<SE3>::Control{
                  .time = START_TIME,
                  .point = TwoJetL<SE3>::identity(),

              },
      },
      ControlPointParameter{
          .value =
              TCurve<SE3>::Control{
                  .time = END_TIME,
                  .point = TwoJetL<SE3>::identity(),
              },
      },
  };

  optimizer.register_error_model("pose_error_model", std::move(error_model));
  optimizer.register_parameters(key, std::move(parameters));

  // ACTION
  const auto result = optimizer.optimize();

  // VERIFICATION
  ASSERT_TRUE(result.converged);

  const auto results = optimizer.get_parameters<ControlPointParameter>(key);

  std::vector<TCurve<SE3>::Control> control_points;
  control_points.reserve(results.data.size());
  for (const auto &point : results.data) {
    control_points.push_back(point.value);
  }

  const TCurve<SE3> t_curve{control_points};

  // For two points, we can expect pretty close or exact matching:
  for (const auto &pose : poses) {
    const auto fit_point = t_curve.point_at(pose.time);
    constexpr double EPSILON = 1e-10;
    EXPECT_NEAR(
        (pose.observation_from_scene * fit_point.frame_from_ref().inverse())
            .log()
            .norm(),
        0.0,
        EPSILON);
  }
}

void save_visualization_log(
    const std::vector<TimedPose> &target_poses,
    const TCurve<SE3> &t_curve) {
  const time::Timestamp start_time{time::as_duration(t_curve.start_time())};
  const time::Timestamp end_time{time::as_duration(t_curve.end_time())};
  McapLogger logger{"vis.mcap"};
  logger.add_proto_channel<::foxglove::PosesInFrame>("observations");
  logger.add_proto_channel<::foxglove::SceneUpdate>("t_curve_path");
  logger.add_proto_channel<::foxglove::PosesInFrame>("escalator_frames");

  ::foxglove::PosesInFrame poses;
  poses.set_frame_id("scene");
  resim::time::proto::pack(start_time, poses.mutable_timestamp());
  for (const auto &pose : target_poses) {
    visualization::foxglove::pack_into_foxglove(
        pose.observation_from_scene.inverse(),
        poses.add_poses());
  }
  logger.log_proto("observations", start_time, poses);

  ::foxglove::SceneUpdate update;
  auto &entity = *update.add_entities();
  resim::time::proto::pack(start_time, entity.mutable_timestamp());
  entity.set_frame_id("scene");
  auto &line = *entity.add_lines();
  line.set_type(::foxglove::LinePrimitive::LINE_STRIP);
  visualization::foxglove::pack_into_foxglove(
      SE3::identity(),
      line.mutable_pose());
  line.set_thickness(2);
  line.set_scale_invariant(true);
  visualization::foxglove::pack_into_foxglove(
      visualization::colors::PERU,
      line.mutable_color());
  constexpr auto DT = 10ms;
  for (auto t = start_time; t < end_time; t += DT) {
    visualization::foxglove::pack_into_foxglove(
        t_curve.point_at(time::as_seconds(t.time_since_epoch()))
            .frame_from_ref()
            .inverse()
            .translation(),
        line.add_points());
  }
  logger.log_proto("t_curve_path", start_time, update);
  const auto spacing = 500ms;
  for (time::Duration outer_t; outer_t < spacing; outer_t += 20ms) {
    ::foxglove::PosesInFrame poses;
    poses.set_frame_id("scene");
    resim::time::proto::pack(start_time + outer_t, poses.mutable_timestamp());
    for (time::Timestamp inner_t = start_time + outer_t; inner_t < end_time;
         inner_t += spacing) {
      visualization::foxglove::pack_into_foxglove(
          t_curve.point_at(time::as_seconds(inner_t.time_since_epoch()))
              .frame_from_ref()
              .inverse(),
          poses.add_poses());
    }
    logger.log_proto("escalator_frames", start_time + outer_t, poses);
  }
}

TEST(PoseErrorModelTest, TestOptimizeComplex) {
  // SETUP
  constexpr int MAX_ITERATIONS = 100;
  constexpr double TOLERANCE = 1e-4;
  math::GaussNewtonOptimizer optimizer{math::GaussNewtonOptimizer::Options{
      .max_iterations = MAX_ITERATIONS,
      .tolerance = TOLERANCE,
  }};
  const std::string key = "trajectory";
  constexpr std::size_t SEED = 893U;
  std::mt19937 rng{SEED};
  constexpr int NUM_POSES = 16;
  std::vector<TimedPose> poses;
  poses.reserve(NUM_POSES);
  constexpr double ELAPSED_TIME = 10.0;
  constexpr double LONG_SPEED = 1.0;
  for (int ii = 0; ii < NUM_POSES; ++ii) {
    const double t = ELAPSED_TIME * static_cast<double>(ii) / (NUM_POSES - 1);
    const double x = LONG_SPEED * t;
    const double y = std::sin(M_PI * x / 4.0);
    const double theta = std::atan2(M_PI * std::cos(M_PI * x / 4.0) / 4.0, 1.0);

    // Create observations with noise
    constexpr double NOISE_MAGNITUDE = 0.05;
    const SE3 observation_from_scene{
        SE3{SO3::exp(theta * Vec3::UnitZ()), Vec3{x, y, 0.0}}.inverse() *
        SE3::exp(
            NOISE_MAGNITUDE *
            testing::random_vector<SE3::TangentVector>(rng).normalized())};

    poses.push_back(TimedPose{
        .time = t,
        .observation_from_scene = observation_from_scene,
    });
  }

  std::unique_ptr<math::ErrorModel> error_model =
      std::make_unique<PoseErrorModel>(optimizer, key, poses);

  std::vector<ControlPointParameter> parameters = {
      ControlPointParameter{
          .value =
              TCurve<SE3>::Control{
                  .time = poses.front().time,
                  .point = TwoJetL<SE3>::identity(),

              },
      },
      ControlPointParameter{
          .value =
              TCurve<SE3>::Control{
                  .time = poses.back().time,
                  .point = TwoJetL<SE3>::identity(),
              },
      },
  };

  optimizer.register_error_model("pose_error_model", std::move(error_model));
  optimizer.register_parameters(key, std::move(parameters));

  // ACTION
  const auto result = optimizer.optimize();

  // VERIFICATION
  EXPECT_TRUE(result.converged);

  const auto results = optimizer.get_parameters<ControlPointParameter>(key);

  std::vector<TCurve<SE3>::Control> control_points;
  control_points.reserve(results.data.size());
  for (const auto &point : results.data) {
    control_points.push_back(point.value);
  }

  const TCurve<SE3> t_curve{control_points};

  save_visualization_log(poses, t_curve);
}

}  // namespace resim::curves::optimization
