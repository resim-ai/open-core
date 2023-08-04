// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>
#include <random>

namespace resim::math {

// Gaussian holds the parameters of a multivariate Gaussian distribution and
// enables the caller to draw samples from the distribution.
class Gaussian {
 public:
  using Vec = Eigen::Matrix<double, Eigen::Dynamic, 1>;
  using Mat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
  using DMat = Eigen::DiagonalMatrix<double, Eigen::Dynamic>;
  using Solver = Eigen::SelfAdjointEigenSolver<Mat>;
  using SamplesMat = Mat;
  using SampleVec = Eigen::Matrix<double, 1, Eigen::Dynamic>;
  // @param[in] mu  - The mean of the Gaussian. This is an dims_ x 1 vector.
  // @param[in] cov - The covariance of the Gaussian. This is an dims_ x dims_
  //                  matrix. cov must be symmetric positive semi-definite.
  //                  Some (non-exhaustive) checks will be performed, but the
  //                  caller is ultimately responsible for providing a well
  //                  formed matrix.
  Gaussian(Vec mu, Mat cov);
  // Draw n samples from the Gaussian.
  // @param[in] n - the number of samples to be drawn.
  // @returns A samples matrix of size n x dims_ with one sample
  //          dims_-dimensional point on each row. Note the return type is a
  //          dynamic-size matrix. If the user knows the number of samples at
  //          compile time then they may choose to return to a static matrix.
  //          However, please note the following: Static matrices live on the
  //          stack and have a restrictive size limit. While the size limit
  //          may be overridden with compile flags this is not advised. Dynamic
  //          matrices are written to the heap and have no such size limit.
  //          For large matrices we recommend sticking with a dynamic type for
  //          the returned matrix, e.g.
  //              Eigen::MatrixXd samp = g.samples(100000);
  SamplesMat samples(unsigned int n);
  // Draw a single sample from the Gaussian.
  // @returns A vector of size 1 x dims_ representing a single dims_-dimensional
  //          point.
  SampleVec sample() { return samples(1); };

  // Getters
  unsigned int dimensionality() const { return dims_; }
  const Vec &mu() const { return mu_; }
  const Mat &cov() const { return cov_; }
  const Vec &eigenvalues() const { return eigenvalues_; }
  const Mat &eigenvectors() const { return eigenvectors_; }
  const DMat &sqrt_eigenvalues_mat() const { return sqrt_eigenvalues_mat_; }

 private:
  // All samples are seeded from a sample over a zero mean, one sigma iid
  // Gaussian. This helper provides these seed samples.
  // @param[in] n - the number of samples to be drawn.
  // @returns A samples matrix of size n x dims_ with one sample
  //          dims_-dimensional point on each row.
  SamplesMat iid_samples(unsigned int n);
  // A natural side effect of solving for eigenvalues is numerical errors
  // leading to small negative eigenvalues. We can tolerate small negative
  // eigenvalues, but we must sanitize them by snapping them to zero because
  // sampling requires taking a square-root.
  // Larger negative eigenvalues are an indication that the user supplied
  // covariance matrix is not valid (symmetric positive semi-definite). In
  // this case we must REASSERT fail.
  void sanitize_eigenvalues();

  // The parameters of the Gaussian.
  // Dimensionality
  const unsigned int dims_;
  // Mean
  Vec mu_{};
  // Covariance
  Mat cov_{};
  // Eigenvalues and eigenvectors.
  Vec eigenvalues_{};
  Mat eigenvectors_{};
  DMat sqrt_eigenvalues_mat_{};

  // Random number generation for sampling.
  // TODO(https://app.asana.com/0/1202178773526279/1203247382049802/f)
  // consider support for non-fixed seeds.
  static constexpr std::size_t SEED = 293U;
  std::mt19937 rng_{SEED};
};

}  // namespace resim::math
