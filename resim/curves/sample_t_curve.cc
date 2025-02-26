// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/sample_t_curve.hh"

#include "resim/assert/assert.hh"
#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/math/multivariate_gaussian.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves {

using resim::transforms::SE3;

namespace {
constexpr int TWO_JET_DOF = optimization::TWO_JET_DOF<SE3>;
}

std::vector<TCurve<SE3>> sample_t_curves(
    const int num_curves,
    const TCurve<SE3> &seed_curve,
    Eigen::VectorXd mean,
    Eigen::MatrixXd covariance) {
  const std::size_t curve_size = seed_curve.control_pts().size();
  REASSERT(
      mean.size() == TWO_JET_DOF * curve_size,
      "Incorrect mean dimension.");
  REASSERT(
      covariance.rows() == TWO_JET_DOF * curve_size,
      "Incorrect covariance dimension.");
  REASSERT(
      covariance.cols() == TWO_JET_DOF * curve_size,
      "Incorrect covariance dimension.");
  math::Gaussian gaussian{std::move(mean), std::move(covariance)};
  return sample_t_curves(num_curves, seed_curve, InOut{gaussian});
}

std::vector<TCurve<SE3>> sample_t_curves(
    const int num_curves,
    const TCurve<SE3> &seed_curve,
    InOut<math::Gaussian> gaussian) {
  const Eigen::MatrixXd samples = gaussian->samples(num_curves);

  std::vector<TCurve<SE3>> results;
  for (int ii = 0; ii < num_curves; ++ii) {
    auto control_points = seed_curve.control_pts();
    for (int jj = 0; jj < control_points.size(); ++jj) {
      auto &point = control_points.at(jj);

      const int block_start = TWO_JET_DOF * jj;
      point.point = optimization::accumulate(
          point.point,
          samples.block<1, TWO_JET_DOF>(ii, block_start).transpose());
    }
    results.emplace_back(control_points);
  }
  return results;
}

}  // namespace resim::curves
