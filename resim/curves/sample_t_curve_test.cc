
#include "resim/curves/sample_t_curve.hh"

#include <gtest/gtest.h>

#include <vector>

#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/curves/t_curve.hh"
#include "resim/transforms/se3.hh"
#include "resim/visualization/curve/visualize_t_curve.hh"

namespace resim::curves {

using transforms::SE3;

void save_visualization_log(const std::vector<TCurve<SE3>> &t_curves) {
  McapLogger logger{"vis.mcap"};

  visualization::curve::MultiTCurveVisualizer visualizer{
      visualization::curve::CurveVisualizationOptions(),
      "/update",
      "/poses",
      InOut(logger)};

  for (const auto &curve : t_curves) {
    visualizer.add_curve(curve);
  }
}

TEST(SampleTCurveTest, TestSampleTCurves) {
  constexpr int NUM_CURVES = 5;

  std::vector<TCurve<SE3>::Control> points;
  for (int ii = 0; ii < 10; ++ii) {
    points.emplace_back(TCurve<SE3>::Control{
        .time = static_cast<double>(ii),
        .point = TwoJetL<SE3>(
            SE3{-ii * Eigen::Vector3d::UnitX()},
            -SE3::TangentVector::Unit(3),
            SE3::TangentVector::Zero()),
    });
  }
  TCurve<SE3> seed_curve{points};

  Eigen::VectorXd mean = Eigen::VectorXd::Zero(optimization::TWO_JET_DOF<SE3>);

  Eigen::MatrixXd cov = 0.0 * Eigen::MatrixXd::Identity(
                                  optimization::TWO_JET_DOF<SE3>,
                                  optimization::TWO_JET_DOF<SE3>);
  cov(4, 4) = 1e-2;

  auto curves = sample_t_curves_pointwise(NUM_CURVES, seed_curve, mean, cov);
  save_visualization_log(curves);
}

}  // namespace resim::curves
