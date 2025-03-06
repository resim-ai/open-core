// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/learning/learn_t_curve_distribution.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <filesystem>
#include <random>
#include <span>

#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/curves/sample_t_curve.hh"
#include "resim/curves/t_curve.hh"
#include "resim/math/is_approx.hh"
#include "resim/time/sample_interval.hh"
#include "resim/visualization/save_visualization_log.hh"

namespace resim::curves::learning {

using transforms::SE3;
using Vec3 = Eigen::Vector3d;
using TwoJetL = TwoJetL<SE3>;
using visualization::save_visualization_log;

// Simple helper to get a covariance matrix that's coerced to be
// positive-semi-definite. Our strategy is to enforce correlation of adjacent
// points and then coerce the result to be positive-semi-definite.
Eigen::MatrixXd covariance(const double magnitude, const int num_points) {
  const int mat_dim = num_points * optimization::TWO_JET_DOF<SE3>;

  constexpr int SEED = 89;
  std::mt19937 rng{SEED};
  constexpr double LB = -0.01;
  constexpr double UB = 0.01;
  std::uniform_real_distribution<double> dist{LB, UB};
  Eigen::MatrixXd L{Eigen::MatrixXd::NullaryExpr(mat_dim, mat_dim, [&]() {
    return dist(rng);
  })};
  return L * L.transpose();
}

std::vector<std::function<StatusValue<TwoJetL>(double)>> make_curves(
    const int num_points,
    const Eigen::MatrixXd &covariance) {
  constexpr int NUM_CURVES = 1000;

  std::vector<TCurve<SE3>::Control> points;
  points.reserve(num_points);
  for (int ii = 0; ii < num_points; ++ii) {
    points.emplace_back(TCurve<SE3>::Control{
        .time = static_cast<double>(ii),
        .point = TwoJetL(
            SE3{-ii * Vec3::UnitX()},
            -SE3::tangent_vector_from_parts(Vec3::Zero(), Vec3::UnitX()),
            SE3::TangentVector::Zero()),
    });
  }
  TCurve<SE3> seed_curve{points};

  const int mat_dim = num_points * optimization::TWO_JET_DOF<SE3>;
  Eigen::VectorXd mean = Eigen::VectorXd::Zero(mat_dim);

  // ACTION
  auto t_curves = sample_t_curves(NUM_CURVES, seed_curve, mean, covariance);

  constexpr int NUM_VIS = 20;
  save_visualization_log(
      std::span(t_curves.cbegin(), t_curves.cbegin() + NUM_VIS),
      "samples.mcap");

  std::vector<std::function<StatusValue<TwoJetL>(double)>> results;
  results.reserve(NUM_CURVES);
  for (auto &t_curve : t_curves) {
    results.emplace_back([t_curve = std::move(t_curve)](const double t) {
      return t_curve.point_at(t);
    });
  }
  return results;
}

TEST(LearnTCurveDistributionTest, TestLearnTCurveDistribution) {
  // SETUP
  constexpr int NUM_POINTS = 5;
  constexpr double START_TIME = 0.0;
  constexpr double END_TIME = NUM_POINTS - 1;
  constexpr double MAX_ABS_DT = 1.0;
  constexpr double MEAN_TOLERANCE = 1e-8;
  constexpr double MEAN_MAX_ITERATIONS = 15;
  std::vector<double> times;
  time::sample_interval(
      START_TIME,
      END_TIME,
      MAX_ABS_DT,
      [&times](const double time) { times.push_back(time); });

  constexpr double MAGNITUDE = 1e-2;
  Eigen::MatrixXd cov = covariance(MAGNITUDE, NUM_POINTS);

  const std::vector<std::function<StatusValue<TwoJetL>(double)>> curves{
      make_curves(NUM_POINTS, cov)};

  // ACTION
  const auto maybe_distribution = learn_t_curve_distribution(
      times,
      curves,
      MEAN_TOLERANCE,
      MEAN_MAX_ITERATIONS);
  ASSERT_TRUE(maybe_distribution.ok());
  const auto &distribution = maybe_distribution.value();

  // VERIFICATION
  ASSERT_TRUE(
      distribution.covariance.isApprox(distribution.covariance.transpose()));
  constexpr double TOLERANCE = 1e-5;
  math::is_approx(cov, distribution.covariance, TOLERANCE);

  save_visualization_log(distribution.mean, "mean.mcap");
}

}  // namespace resim::curves::learning
