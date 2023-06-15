#pragma once

#include <Eigen/Dense>

#include "resim/assert/assert.hh"

namespace resim::math {

// Mean over N (D-dimensional) samples.
// @param[in] - samples is an N x D matrix representing N samples over a
//              D-dimensional distribution.
// @returns A D x 1 vector represention the mean point of the sample
//          distribution.
Eigen::VectorXd mean(const Eigen::MatrixXd &samples) {
  return samples.colwise().mean();
}

// Covariance over N (D-dimensional) samples.
// @param[in] - samples is an N x D matrix representing N samples over a
//              D-dimensional distribution.
// @returns A D x D matrix represention the covariance  of the sample
//          distribution.
Eigen::MatrixXd covariance(const Eigen::MatrixXd &samples) {
  const unsigned int sample_count = samples.rows();
  constexpr auto ERR_MSG =
      "At least two samples are needed to compute a covariance matrix";
  REASSERT(sample_count > 1, ERR_MSG);
  const Eigen::MatrixXd zero_mean =
      samples.rowwise() - samples.colwise().mean();
  Eigen::MatrixXd cov =
      (zero_mean.adjoint() * zero_mean) / static_cast<double>(sample_count - 1);
  return cov;
}

}  // namespace resim::math
