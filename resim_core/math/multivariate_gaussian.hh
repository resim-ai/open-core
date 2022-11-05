#pragma once

#include <glog/logging.h>

#include <Eigen/Dense>
#include <algorithm>
#include <random>
#include <utility>

namespace resim::math {

// Gaussian holds the parameters of a multivariate Gaussian distribution and
// enables the caller to draw samples from the distribution. Although a dynamic
// implementation is possible, we have chosen to template this class to the
// dimensionality of the Gaussian. We expect that the dimensionality is known
// at compile time and being explicit allows us to make more checks and avoid
// pesky bugs, e.g  where the number of samples and dimensionality become
// confused.
template <unsigned int dims>
class Gaussian {
 public:
  // The dimensionality of the Gaussian distribution.
  static constexpr unsigned int DIMS = dims;
  using Vec = Eigen::Matrix<double, DIMS, 1>;
  using Mat = Eigen::Matrix<double, DIMS, DIMS>;
  using DMat = Eigen::DiagonalMatrix<double, DIMS>;
  using Solver = Eigen::SelfAdjointEigenSolver<Mat>;
  using SamplesMat = Eigen::Matrix<double, Eigen::Dynamic, DIMS>;
  // @param[in] mu  - The mean of the Gaussian. This is an DIMS x 1 vector.
  // @param[in] cov - The covariance of the Gaussian. This is an DIMS x DIMS
  //                  matrix. cov must be symmetric positive semi-definite.
  //                  Some (non-exhaustive) checks will be performed, but the
  //                  caller is ultimately responsible for providing a well
  //                  formed matrix.
  Gaussian(Vec mu, Mat cov);
  // Draw n samples from the Gaussian.
  // @param[in] n - the number of samples to be drawn.
  // @returns A samples matrix of size n x DIMS with one sample DIMS-dimensional
  //          point on each row. Note the return type is a dynamic-size matrix.
  //          If the user knows the number of samples at compile time then they
  //          may choose to return to a static matrix. However, please note the
  //          following: Static matrices live on the stack and have a
  //          restrictive size limit. While the size limit may be overridden
  //          with compile flags this is not advised. Dynamic matrices are
  //          written to the heap and have no such size limit.
  //          For large matrices we recommend sticking with a dynamic type for
  //          the returned matrix, e.g.
  //              Eigen::MatrixXd samp = g.samples(100000);
  SamplesMat samples(unsigned int n);
  // Draw a single sample from the Gaussian.
  // @returns A vector of size 1 x DIMS representing a single DIMS-dimensional
  //          point.
  Eigen::Matrix<double, 1, DIMS> sample() { return samples(1); };

  // Getters
  const Vec &mu() const { return mu_; }
  const Mat &cov() const { return cov_; }
  const Vec &eigenvalues() const { return eigenvalues_; }
  const Mat &eigenvectors() const { return eigenvectors_; }
  const DMat &sqrt_eigenvalues_mat() const { return sqrt_eigenvalues_mat_; }

 private:
  // All samples are seeded from a sample over a zero mean, one sigma iid
  // Gaussian. This helper provides these seed samples.
  // @param[in] n - the number of samples to be drawn.
  // @returns A samples matrix of size n x DIMS with one sample DIMS-dimensional
  //          point on each row.
  SamplesMat iid_samples(unsigned int n);
  // A natural side effect of solving for eigenvalues is numerical errors
  // leading to small negative eigenvalues. We can tolerate small negative
  // eigenvalues, but we must sanitize them by snapping them to zero because
  // sampling requires taking a square-root.
  // Larger negative eigenvalues are an indication that the user supplied
  // covariance matrix is not valid (symmetric positive semi-definite). In
  // this case we must CHECK fail.
  void sanitize_eigenvalues();

  // The parameters of the Gaussian.
  Vec mu_{Vec::Zero()};
  Mat cov_{Mat::Identity()};
  // Eigenvalues and eigenvectors.
  Vec eigenvalues_{Vec::Ones()};
  Mat eigenvectors_{Mat::Identity()};
  DMat sqrt_eigenvalues_mat_{Vec::Ones()};

  // Random number generation for sampling.
  // TODO(https://app.asana.com/0/1202178773526279/1203247382049802/f)
  // consider support for non-fixed seeds.
  static constexpr std::size_t SEED = 293U;
  std::mt19937 rng_{SEED};
};

template <unsigned int dims>
Gaussian<dims>::Gaussian(Vec mu, Mat cov)
    : mu_(std::move(mu)),
      cov_(std::move(cov)) {
  // Covariance matrices should be symmetric positive semi-definite.
  // We do not check this exhaustively but do a number of supporting checks.
  // Specifically for symmetry and positive eigenvalues. We also check for
  // numerical issues in the Eigen solver.
  constexpr auto SYMMETRY_ERR = "Covariance matrix must be symmetric";
  CHECK(cov_.isApprox(cov_.transpose())) << SYMMETRY_ERR;
  Solver solver(cov_);
  CHECK(solver.info() != Eigen::NumericalIssue) << "Eigen solver error.";
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

template <unsigned int dims>
typename Gaussian<dims>::SamplesMat Gaussian<dims>::samples(unsigned int n) {
  const SamplesMat coeffs = this->iid_samples(n) * sqrt_eigenvalues_mat_;
  return (coeffs * eigenvectors_.transpose()).rowwise() + mu_.transpose();
}

template <unsigned int dims>
typename Gaussian<dims>::SamplesMat Gaussian<dims>::iid_samples(
    unsigned int n) {
  std::mt19937 &rng = rng_;  // Ref to avoid capturing 'this' in the lambda
  std::normal_distribution<double> norm(0., 1.);
  const auto rand_norm = [&]() { return norm(rng); };
  return SamplesMat::NullaryExpr(n, DIMS, rand_norm);
}

template <unsigned int dims>
void Gaussian<dims>::sanitize_eigenvalues() {
  constexpr double TOL_BASE = 1e-15;
  // We need to scale our tolerance according to the scale of the covariance
  // matrix. Broadly, covariance matrices with wider ranging values deserve
  // wider tolerance. There may be a more principled way to do this than the
  // method below, however this performs sufficiently well on our test cases.
  const double tol =
      std::max((cov_.maxCoeff() - cov_.minCoeff()) * TOL_BASE, TOL_BASE);
  constexpr auto EIGENVALS_ERR =
      "Symmetric PSD matrices have positive eigenvalues";
  CHECK(eigenvalues_.minCoeff() > -tol) << EIGENVALS_ERR;
  eigenvalues_ = eigenvalues_.unaryExpr(
      [&](double eigenval) { return std::max(eigenval, 0.); });
}

}  // namespace resim::math
