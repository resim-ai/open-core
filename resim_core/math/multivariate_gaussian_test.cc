#include "resim_core/math/multivariate_gaussian.hh"

#include <asm-generic/errno.h>
#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <random>
#include <type_traits>

#include "resim_core/assert/assert.hh"
#include "resim_core/math/sample_statistics.hh"

namespace resim::math {

namespace {
// Some useful dimensionalities for testing.
constexpr unsigned int THREE_D = 3;
constexpr unsigned int SIX_D = 6;
constexpr unsigned int EIGHTEEN_D = 18;

// Some useful multipliers for testing.
constexpr double SML = 1.32E-4;
constexpr double ONE = 1.;
constexpr double LRG = 6.1E5;
constexpr std::array<double, 3> SCALES{SML, ONE, LRG};
}  // namespace

class GaussianTest : public ::testing::TestWithParam<unsigned int> {
 protected:
  struct GaussParams {
    Eigen::VectorXd mu;
    Eigen::MatrixXd cov;
  };

  static GaussParams identity_parameters(unsigned int dims) {
    return {
        Eigen::MatrixXd::Zero(dims, 1),
        Eigen::MatrixXd::Identity(dims, dims)};
  }

  Eigen::MatrixXd generate_randn_matrix(unsigned int rows, unsigned int cols) {
    std::mt19937 &rng = rng_;  // Ref to avoid capturing 'this' in the lambda
    std::normal_distribution<double> norm(0, 1);
    auto rand_norm = [&]() { return norm(rng); };
    Eigen::MatrixXd rand_matrix =
        Eigen::MatrixXd::NullaryExpr(rows, cols, rand_norm);
    return rand_matrix;
  }

  GaussParams generate_parameters(unsigned int dims, double multiplier = 1.) {
    // The simplest way to guarantee that a generated mean vector and
    // covariance matrix are valid is to compute them over a set of samples.
    constexpr unsigned int SAMPLE_COUNT = 2;
    Eigen::MatrixXd rand_samples = generate_randn_matrix(SAMPLE_COUNT, dims);
    rand_samples *= multiplier;
    return {mean(rand_samples), covariance(rand_samples)};
  }

 private:
  static constexpr std::size_t SEED = 293U;
  std::mt19937 rng_{SEED};
};

TEST_P(GaussianTest, ConstructionMeanCov) {
  // Check that the Gaussian can be constructed and parameters can be set
  // and retrieved.
  // Generate random parameters.
  const unsigned int D = GetParam();
  const typename GaussianTest::GaussParams p = this->generate_parameters(D);
  // Create the Gaussian.
  const Gaussian g(p.mu, p.cov);
  // Check the retrieved parameters match.
  EXPECT_EQ(D, g.dimensionality());
  EXPECT_EQ(p.mu, g.mu());
  EXPECT_EQ(p.cov, g.cov());
}

TEST_P(GaussianTest, ConstructionDimensionality) {
  // Check that - following construction - the dimensionality of gettable data
  // and samples are consistent with the dimensionality of the Gaussian.
  // Generate random parameters.
  const unsigned int D = GetParam();
  const typename GaussianTest::GaussParams p = this->generate_parameters(D);
  // Create the Gaussian.
  Gaussian g(p.mu, p.cov);
  // Check the dimensionalities of gettable data.
  EXPECT_EQ(g.mu().rows(), D);
  EXPECT_EQ(g.mu().cols(), 1);
  EXPECT_EQ(g.cov().rows(), D);
  EXPECT_EQ(g.cov().cols(), D);
  EXPECT_EQ(g.eigenvalues().rows(), D);
  EXPECT_EQ(g.eigenvalues().cols(), 1);
  EXPECT_EQ(g.eigenvectors().rows(), D);
  EXPECT_EQ(g.eigenvectors().cols(), D);
  EXPECT_EQ(g.sqrt_eigenvalues_mat().rows(), D);
  EXPECT_EQ(g.sqrt_eigenvalues_mat().cols(), D);
  // Check the dimensionalities of gettable samples.
  constexpr unsigned int SAMPLE_N = 5;
  const Eigen::MatrixXd samples = g.samples(SAMPLE_N);
  EXPECT_EQ(samples.rows(), SAMPLE_N);
  EXPECT_EQ(samples.cols(), D);
}

TEST_P(GaussianTest, ConstructionTrivialEigenValues) {
  // For a one sigma iid Gaussian, the Eigenvalues are all 1 and the
  // Eigenvectors matrix is equal to the covariance matrix, which is equal to
  // identity.
  const unsigned int D = GetParam();
  const typename GaussianTest::GaussParams p =
      GaussianTest::identity_parameters(D);
  const Gaussian g(p.mu, p.cov);
  EXPECT_TRUE(Eigen::VectorXd::Ones(D).isApprox(g.eigenvalues()));
  EXPECT_TRUE(p.cov.isApprox(g.eigenvectors()));
  EXPECT_TRUE(Eigen::MatrixXd::Identity(D, D).isApprox(g.eigenvectors()));
}

TEST_P(GaussianTest, ConstructionSqrtEigenValues) {
  // For a trivial diagonal covariance matrix with 2's on the diagonal.
  // The eigenvalues are all 2 so the sqrt eigenvalues matrix is:
  //     identity x sqrt(2).
  const unsigned int D = GetParam();
  typename GaussianTest::GaussParams p = GaussianTest::identity_parameters(D);
  constexpr double TWO = 2.;
  p.cov *= TWO;
  Eigen::MatrixXd expected_diagonal{Eigen::VectorXd::Ones(D) * sqrt(TWO)};
  const Gaussian g(p.mu, p.cov);
  EXPECT_TRUE(g.sqrt_eigenvalues_mat().diagonal().isApprox(expected_diagonal));
}

TEST_P(GaussianTest, SmallNegativeEigenvaluesFix) {
  // If the computed eigenvalues are negative and small enough to be forgiven
  // as numerical precision errors, then we expect them to be snapped to zero.
  const unsigned int D = GetParam();
  typename GaussianTest::GaussParams p = GaussianTest::identity_parameters(D);
  constexpr double TINY_NEG = -1e-16;
  p.cov *= TINY_NEG;
  Gaussian g(p.mu, p.cov);
  EXPECT_TRUE(g.eigenvalues().isZero());
}

TEST_P(GaussianTest, Sampling) {
  // If we generate a significant number of samples then we can approximate the
  // parameters of the Gaussian distribution by computing the mean and the
  // covariance over the sample set, thus demonstrating the correctness of the
  // sampling method.
  const unsigned int D = GetParam();
  constexpr unsigned int SAMPLE_N = 50000;
  constexpr unsigned int TRIES = 11;
  // We are only approximating the parameters so low precision is expected.
  constexpr double PREC = 0.02;
  for (double scale : SCALES) {
    for (int i = 0; i < TRIES; ++i) {
      const typename GaussianTest::GaussParams p =
          this->generate_parameters(D, scale);
      Gaussian g(p.mu, p.cov);
      // Draw samples
      const Eigen::MatrixXd samples = g.samples(SAMPLE_N);
      // Compute the mean
      const Eigen::VectorXd mu_c = mean(samples);
      // Compute the covariance
      const Eigen::MatrixXd cov_c = covariance(samples);
      // Compare the computed statistics to the parameters of the distribution
      // being sampled.
      EXPECT_TRUE(mu_c.isApprox(g.mu(), PREC));
      EXPECT_TRUE(cov_c.isApprox(g.cov(), PREC));
    }
  }
}

TEST_P(GaussianTest, SingleSampling) {
  // Gaussian::sample() is simply a convenience overload of samples(1).
  // Given that samples(n) is tested above, this test simply asserts that
  // samples returns a single row matrix (vector) as expected.
  const unsigned int D = GetParam();
  const typename GaussianTest::GaussParams p = this->generate_parameters(D);
  Gaussian g(p.mu, p.cov);
  Eigen::MatrixXd sample;
  EXPECT_NO_THROW({
    // Try to assign the sample to a vector of expected dimensionality.
    sample = g.sample();
  });
}

TEST_P(GaussianTest, SingularMatrix) {
  // The simplest singular matrix is a matrix of zeros.
  const unsigned int D = GetParam();
  const Eigen::VectorXd mu = Eigen::VectorXd::Zero(D);
  const Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(D, D);
  // Build Gaussian of zeros.
  Gaussian g(mu, cov);
  // Sample should be all zeros.
  Eigen::MatrixXd s = g.sample();
  EXPECT_TRUE(s.isZero());
}

using GaussianDeathTest = GaussianTest;

TEST_P(GaussianDeathTest, DimensiomalMismatch) {
  // NOLINTBEGIN(readability-function-cognitive-complexity)
  // Generate a mean and a covariance with mismatched dimensionality
  const unsigned int D = GetParam();
  const Eigen::VectorXd mu = Eigen::VectorXd::Zero(D);
  const unsigned int WRONG_D = D + 1;
  constexpr unsigned int COMBINATIONS = 3;
  const std::array<Eigen::MatrixXd, COMBINATIONS> covariances{
      this->generate_randn_matrix(WRONG_D, D),
      this->generate_randn_matrix(D, WRONG_D),
      this->generate_randn_matrix(WRONG_D, WRONG_D)};

  for (const Eigen::MatrixXd &cov : covariances) {
    EXPECT_THROW(
        {
          Gaussian g(mu, cov);
          (void)g;  // Avoid unused variable errors.
        },
        AssertException);
  }
  // NOLINTEND(readability-function-cognitive-complexity)
}

TEST_P(GaussianDeathTest, NonSymmetricCov) {
  const unsigned int D = GetParam();
  const Eigen::VectorXd mu = Eigen::VectorXd::Zero(D);
  const Eigen::MatrixXd cov = this->generate_randn_matrix(D, D);
  // Check that random did not accidentally generate a symmetric matrix.
  ASSERT_FALSE(cov.isApprox(cov.transpose()));
  EXPECT_THROW(
      {
        Gaussian g(mu, cov);
        (void)g;  // Avoid unused variable errors.
      },
      AssertException);
}

TEST_P(GaussianDeathTest, NegativeEigenvalues) {
  const unsigned int D = GetParam();
  const Eigen::VectorXd mu = Eigen::VectorXd::Zero(D);
  // Negative numbers on the diagonal will result in negative eigenvalues.
  const Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(D, D) * -1;
  EXPECT_THROW(
      {
        Gaussian g(mu, cov);
        (void)g;  // Avoid unused variable errors.
      },
      AssertException);
}

INSTANTIATE_TEST_SUITE_P(
    GaussianTestParametrized,
    GaussianTest,
    ::testing::Values(THREE_D, SIX_D, EIGHTEEN_D));

INSTANTIATE_TEST_SUITE_P(
    GaussianDeathTestParametrized,
    GaussianDeathTest,
    ::testing::Values(THREE_D, SIX_D, EIGHTEEN_D));

}  // namespace resim::math
