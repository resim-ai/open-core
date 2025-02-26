
#include "resim/curves/sample_t_curve.hh"

#include <gtest/gtest.h>

#include <filesystem>
#include <vector>

#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/curves/t_curve.hh"
#include "resim/transforms/se3.hh"
#include "resim/visualization/curve/visualize_t_curve.hh"

namespace resim::curves {

using Vec3 = Eigen::Vector3d;
using transforms::SE3;

// Simple visualization for the tcurves
void save_visualization_log(const std::vector<TCurve<SE3>> &t_curves) {
  const char *maybe_outputs_dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");
  const std::filesystem::path OUTPUTS_DIR{
      maybe_outputs_dir ? maybe_outputs_dir : "."};
  resim::McapLogger logger{OUTPUTS_DIR / "vis.mcap"};

  visualization::curve::MultiTCurveVisualizer visualizer{
      visualization::curve::CurveVisualizationOptions(),
      "/update",
      "/poses",
      InOut(logger)};

  for (const auto &curve : t_curves) {
    visualizer.add_curve(curve);
  }
}

// Simple helper to get a covariance matrix that's coerced to be
// positive-semi-definite. Our strategy is to enforce correlation of adjacent
// points and then coerce the result to be positive-semi-definite.
Eigen::MatrixXd covariance(
    const double magnitude,
    const std::size_t num_points) {
  Eigen::MatrixXd cov{Eigen::MatrixXd::Zero(
      num_points * optimization::TWO_JET_DOF<SE3>,
      num_points * optimization::TWO_JET_DOF<SE3>)};
  constexpr auto DOF = optimization::TWO_JET_DOF<SE3>;
  for (int ii = 0; ii < (num_points - 1u); ++ii) {
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
  Eigen::MatrixXd eigenvectors{solver.eigenvectors()};
  return eigenvectors * coerced_diagonal * eigenvectors.inverse();
}

TEST(SampleTCurveTest, TestSampleTCurves) {
  // SETUP
  constexpr int NUM_CURVES = 5;

  std::vector<TCurve<SE3>::Control> points;
  constexpr int NUM_POINTS = 10;
  for (int ii = 0; ii < NUM_POINTS; ++ii) {
    points.emplace_back(TCurve<SE3>::Control{
        .time = static_cast<double>(ii),
        .point = TwoJetL<SE3>(
            SE3{-ii * Vec3::UnitX()},
            -SE3::tangent_vector_from_parts(Vec3::Zero(), Vec3::UnitX()),
            SE3::TangentVector::Zero()),
    });
  }
  TCurve<SE3> seed_curve{points};

  Eigen::VectorXd mean =
      Eigen::VectorXd::Zero(NUM_POINTS * optimization::TWO_JET_DOF<SE3>);
  constexpr double MAGNITUDE = 1e-2;
  Eigen::MatrixXd cov = covariance(MAGNITUDE, NUM_POINTS);

  // ACTION
  auto curves = sample_t_curves(NUM_CURVES, seed_curve, mean, cov);

  // VERIFICATION
  EXPECT_EQ(curves.size(), NUM_CURVES);
  for (const auto &curve : curves) {
    ASSERT_EQ(curve.control_pts().size(), NUM_POINTS);
    for (int ii = 0; ii < NUM_POINTS; ++ii) {
      const Eigen::Vector<double, optimization::TWO_JET_DOF<SE3>> diff =
          optimization::difference(
              curve.control_pts().at(ii).point,
              seed_curve.control_pts().at(ii).point);

      constexpr double EPSILON = 1e-10;
      ASSERT_GT(diff.norm(), EPSILON);

      constexpr double UNLIKELY_TO_BE_THIS_BIG = 1.0;
      ASSERT_LT(diff.norm(), UNLIKELY_TO_BE_THIS_BIG);
    }
  }
  save_visualization_log(curves);
}

}  // namespace resim::curves
