// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/learning/learn_t_curve_distribution.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/curves/sample_t_curve.hh"
#include "resim/curves/t_curve.hh"
#include "resim/time/sample_interval.hh"

namespace resim::curves::learning {

using transforms::SE3;
using Vec3 = Eigen::Vector3d;
using TwoJetL = TwoJetL<SE3>;

// Simple helper to get a covariance matrix that's coerced to be
// positive-semi-definite. Our strategy is to enforce correlation of adjacent
// points and then coerce the result to be positive-semi-definite.
Eigen::MatrixXd covariance(const double magnitude, const int num_points) {
  const int mat_dim = num_points * optimization::TWO_JET_DOF<SE3>;

  Eigen::MatrixXd cov{Eigen::MatrixXd::Zero(mat_dim, mat_dim)};
  constexpr auto DOF = optimization::TWO_JET_DOF<SE3>;
  for (int ii = 0; ii < (num_points - 1); ++ii) {
    for (int jj = 0; jj < DOF; ++jj) {
      cov(ii * DOF + jj, (ii + 1) * DOF + jj) = magnitude;
      cov((ii + 1) * DOF + jj, ii * DOF + jj) = magnitude;
    }
  }
  // Coerce PSD:
  using Solver = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>;
  Solver solver(cov);
  REASSERT(solver.info() != Eigen::NumericalIssue, "Eigen solver error.");

  Eigen::DiagonalMatrix<double, Eigen::Dynamic> coerced_diagonal{
      solver.eigenvalues()
          .unaryExpr([&](double eigenval) { return std::max(eigenval, 0.); })
          .asDiagonal()};
  const Eigen::MatrixXd &eigenvectors{solver.eigenvectors()};
  return eigenvectors * coerced_diagonal * eigenvectors.inverse();
}

std::vector<TCurve<SE3>> make_curves() {
  constexpr int NUM_CURVES = 5;

  constexpr int NUM_POINTS = 10;
  std::vector<TCurve<SE3>::Control> points;
  points.reserve(NUM_POINTS);
  for (int ii = 0; ii < NUM_POINTS; ++ii) {
    points.emplace_back(TCurve<SE3>::Control{
        .time = static_cast<double>(ii),
        .point = TwoJetL(
            SE3{-ii * Vec3::UnitX()},
            -SE3::tangent_vector_from_parts(Vec3::Zero(), Vec3::UnitX()),
            SE3::TangentVector::Zero()),
    });
  }
  TCurve<SE3> seed_curve{points};

  constexpr int MAT_DIM = NUM_POINTS * optimization::TWO_JET_DOF<SE3>;
  Eigen::VectorXd mean = Eigen::VectorXd::Zero(MAT_DIM);
  constexpr double MAGNITUDE = 1e-2;
  Eigen::MatrixXd cov = covariance(MAGNITUDE, NUM_POINTS);

  // ACTION
  return sample_t_curves(NUM_CURVES, seed_curve, mean, cov);
}

TEST(LearnTCurveDistributionTest, TestLearnTCurveDistribution) {
  // SETUP
  constexpr double START_TIME = 0.0;
  constexpr double END_TIME = 10.0;
  constexpr double MAX_ABS_DT = 1.5;
  std::vector<double> times;
  time::sample_interval(
      START_TIME,
      END_TIME,
      MAX_ABS_DT,
      [&times](const double time) { times.push_back(time); });

  std::vector<std::function<StatusValue<TwoJetL>(double)>> curves;
  curves.emplace_back([](const double t) -> StatusValue<TwoJetL> {
    return TwoJetL::identity();
  });

  for (int ii = 0; ii < 10; ++ii) {
    curves.emplace_back(curves.back());
  }

  // ACTION

  auto maybe_distribution = learn_t_curve_distribution(times, curves);

  ASSERT_TRUE(maybe_distribution.ok());
}

}  // namespace resim::curves::learning
