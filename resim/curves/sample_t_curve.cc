

#include "resim/curves/sample_t_curve.hh"

#include "resim/assert/assert.hh"
#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/math/multivariate_gaussian.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves {

using resim::transforms::SE3;

std::vector<TCurve<SE3>> sample_t_curves_pointwise(
    const int num_curves,
    const TCurve<SE3> &seed_curve,
    Eigen::VectorXd mean,
    Eigen::MatrixXd covariance) {
  constexpr int TWO_JET_DOF = optimization::TWO_JET_DOF<SE3>;
  REASSERT(mean.size() == TWO_JET_DOF, "Incorrect mean dimension.");
  REASSERT(covariance.rows() == TWO_JET_DOF, "Incorrect covariance dimension.");
  REASSERT(covariance.cols() == TWO_JET_DOF, "Incorrect covariance dimension.");
  math::Gaussian gaussian{std::move(mean), std::move(covariance)};

  const Eigen::MatrixXd samples =
      gaussian.samples(num_curves * seed_curve.control_pts().size());

  std::vector<TCurve<SE3>> results;
  for (int ii = 0; ii < num_curves; ++ii) {
    auto control_points = seed_curve.control_pts();
    for (int jj = 0; jj < control_points.size(); ++jj) {
      auto &point = control_points.at(jj);
      point.point = optimization::accumulate(
          point.point,
          samples.row(ii * control_points.size() + jj).transpose());
    }
    results.emplace_back(control_points);
  }
  return results;
}

}  // namespace resim::curves
