#include "resim/math/sample_statistics.hh"

#include <gtest/gtest.h>

#include "resim/assert/assert.hh"

namespace resim::math {

namespace {
const Eigen::Vector3d TEST_VEC{0., 1., 2.};
}  // namespace

TEST(MeanTest, TrivialMean) {
  // Create a sample matrix with mean one..
  Eigen::Matrix<double, 3, 2> samples;
  samples.col(0) << TEST_VEC;
  samples.col(1) << TEST_VEC;
  // Confim that mean is (1,1).
  const Eigen::Vector2d mu = mean(samples);
  EXPECT_TRUE(mu.isApprox(Eigen::Vector2d::Ones()));
}

TEST(CovarianceTest, PerfectCorrelation) {
  // Create a sample matrix with perfect correlation.
  Eigen::Matrix<double, 3, 2> samples;
  samples.col(0) << TEST_VEC;
  samples.col(1) << TEST_VEC;

  const Eigen::Matrix2d cov = covariance(samples);
  // Config covariance matrix is all ones.
  EXPECT_TRUE(cov.isApprox(Eigen::Matrix2d::Ones()));
}

TEST(CovarianceTest, PerfectAntiCorrelation) {
  // Create a sample matrix with perfect anti-correlation.
  Eigen::Matrix<double, 3, 2> samples;
  samples.col(0) << TEST_VEC;
  samples.col(1) << TEST_VEC.reverse();

  const Eigen::Matrix2d cov = covariance(samples);
  Eigen::Matrix2d expected_cov;
  expected_cov << 1., -1., -1., 1;
  // Confirm covariance matrix is as expected.
  EXPECT_TRUE(cov.isApprox(expected_cov));
}

TEST(CovarianceDeathTest, TooFewSamples) {
  // Create a matrix with only one sample.
  const Eigen::Matrix<double, 1, 2> samples =
      Eigen::Matrix<double, 1, 2>::Zero();
  EXPECT_THROW(
      {
        const Eigen::Matrix2d cov = covariance(samples);
        (void)cov;  // Avoid unused variable errors.
      },
      AssertException);
}

}  // namespace resim::math
