#include "resim_core/math/multivariate_gaussian.hh"

#include <glog/logging.h>

#include <Eigen/Dense>
#include <random>
#include <utility>

#include "resim_core/assert/assert.hh"

namespace resim::math {

Gaussian::Gaussian(Vec mu, Mat cov)
    : dims_(mu.rows()),
      mu_(std::move(mu)),
      cov_(std::move(cov)) {
  // Check that covariance has the same dimensionality as the mean.
  constexpr auto DIM_ERR =
      "Mean and covariance should have the same dimensionality";
  REASSERT((cov_.rows() == dims_ and cov_.cols() == dims_), DIM_ERR);
  // Covariance matrices should be symmetric positive semi-definite.
  // We do not check this exhaustively but do a number of supporting checks.
  // Specifically for symmetry and positive eigenvalues. We also check for
  // numerical issues in the Eigen solver.
  constexpr auto SYMMETRY_ERR = "Covariance matrix must be symmetric";
  REASSERT(cov_.isApprox(cov_.transpose()), SYMMETRY_ERR);
  Solver solver(cov_);
  REASSERT(solver.info() != Eigen::NumericalIssue, "Eigen solver error.");
  eigenvalues_ = solver.eigenvalues();
  // If the eigenvalues are close to zero, the solver can return small
  // negative eigenvalues, these will cause problems later on. These must be
  // sanitized.
  sanitize_eigenvalues();
  eigenvectors_ = solver.eigenvectors();
  // Note: Clang tidy wants this initialized in an initialization list,
  // despite its dependence on eigenvalues_.
  // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
  sqrt_eigenvalues_mat_ = eigenvalues_.cwiseSqrt().asDiagonal();
}

typename Gaussian::SamplesMat Gaussian::samples(unsigned int n) {
  const SamplesMat coeffs = this->iid_samples(n) * sqrt_eigenvalues_mat_;
  return (coeffs * eigenvectors_.transpose()).rowwise() + mu_.transpose();
}

typename Gaussian::SamplesMat Gaussian::iid_samples(unsigned int n) {
  std::mt19937 &rng = rng_;  // Ref to avoid capturing 'this' in the lambda
  std::normal_distribution<double> norm(0., 1.);
  const auto rand_norm = [&]() { return norm(rng); };
  return SamplesMat::NullaryExpr(n, dims_, rand_norm);
}

void Gaussian::sanitize_eigenvalues() {
  constexpr double TOL_BASE = 1e-15;
  // We need to scale our tolerance according to the scale of the covariance
  // matrix. Broadly, covariance matrices with wider ranging values deserve
  // wider tolerance. There may be a more principled way to do this than the
  // method below, however this performs sufficiently well on our test cases.
  const double tol =
      std::max((cov_.maxCoeff() - cov_.minCoeff()) * TOL_BASE, TOL_BASE);
  constexpr auto EIGENVALS_ERR =
      "Symmetric PSD matrices have positive eigenvalues";
  REASSERT(eigenvalues_.minCoeff() > -tol, EIGENVALS_ERR);
  eigenvalues_ = eigenvalues_.unaryExpr(
      [&](double eigenval) { return std::max(eigenval, 0.); });
}

}  // namespace resim::math
