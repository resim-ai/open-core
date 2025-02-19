// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/optimization/pose_error_model.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <utility>
#include <vector>

#include "resim/assert/assert.hh"
#include "resim/curves/optimization/control_point_parameter.hh"
#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves::optimization {

using transforms::SE3;
using Vec3 = Eigen::Vector3d;

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

}  // namespace resim::curves::optimization
