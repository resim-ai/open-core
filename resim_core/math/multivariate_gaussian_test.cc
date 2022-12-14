#include "resim_core/math/multivariate_gaussian.hh"

#include <asm-generic/errno.h>
#include <gtest/gtest.h>

#include <random>
#include <type_traits>

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

template <typename T>
class GaussianTest : public ::testing::Test {
 protected:
  struct GaussParams {
    Eigen::VectorXd mu;
    Eigen::MatrixXd cov;
  };

  using VectorD = Eigen::Matrix<double, T::value, 1>;
  using MatrixDD = Eigen::Matrix<double, T::value, T::value>;

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

using DimNonTypes = ::testing::Types<
    std::integral_constant<unsigned int, THREE_D>,
    std::integral_constant<unsigned int, SIX_D>,
    std::integral_constant<unsigned int, EIGHTEEN_D>>;

TYPED_TEST_SUITE(GaussianTest, DimNonTypes);

TYPED_TEST(GaussianTest, ConstructionMeanCov) {
  // Check that the Gaussian can be constructed and parameters can be set
  // and retrieved.
  // Generate random parameters.
  constexpr unsigned int D = TypeParam::value;
  const typename GaussianTest<TypeParam>::GaussParams p =
      this->generate_parameters(D);
  // Create the Gaussian.
  const Gaussian<D> g(p.mu, p.cov);
  // Check the retrieved parameters match.
  EXPECT_EQ(p.mu, g.mu());
  EXPECT_EQ(p.cov, g.cov());
}

TYPED_TEST(GaussianTest, ConstructionTrivialEigenValues) {
  // For a one sigma iid Gaussian, the Eigenvalues are all 1 and the
  // Eigenvectors matrix is equal to the covariance matrix, which is equal to
  // identity.
  constexpr unsigned int D = TypeParam::value;
  const typename GaussianTest<TypeParam>::GaussParams p =
      this->identity_parameters(D);
  const Gaussian<D> g(p.mu, p.cov);
  EXPECT_TRUE(
      GaussianTest<TypeParam>::VectorD::Ones().isApprox(g.eigenvalues()));
  EXPECT_TRUE(p.cov.isApprox(g.eigenvectors()));
  EXPECT_TRUE(
      GaussianTest<TypeParam>::MatrixDD::Identity().isApprox(g.eigenvectors()));
}

TYPED_TEST(GaussianTest, ConstructionSqrtEigenValues) {
  // For a trivial diagonal covariance matrix with 2's on the diagonal.
  // The eigenvalues are all 2 so the sqrt eigenvalues matrix is:
  //     identity x sqrt(2).
  constexpr unsigned int D = TypeParam::value;
  typename GaussianTest<TypeParam>::GaussParams p =
      this->identity_parameters(D);
  constexpr double TWO = 2.;
  p.cov *= TWO;
  Eigen::Matrix<double, D, 1> expected_diagonal{
      Eigen::Matrix<double, D, 1>::Ones() * sqrt(TWO)};
  const Gaussian<D> g(p.mu, p.cov);
  EXPECT_TRUE(g.sqrt_eigenvalues_mat().diagonal().isApprox(expected_diagonal));
}

TYPED_TEST(GaussianTest, SmallNegativeEigenvaluesFix) {
  // If the computed eigenvalues are negative and small enough to be forgiven
  // as numerical precision errors, then we expect them to be snapped to zero.
  constexpr unsigned int D = TypeParam::value;
  typename GaussianTest<TypeParam>::GaussParams p =
      this->identity_parameters(D);
  constexpr double TINY_NEG = -1e-16;
  p.cov *= TINY_NEG;
  Gaussian<D> g(p.mu, p.cov);
  EXPECT_TRUE(g.eigenvalues().isZero());
}

TYPED_TEST(GaussianTest, Sampling) {
  // If we generate a significant number of samples then we can approximate the
  // parameters of the Gaussian distribution by computing the mean and the
  // covariance over the sample set, thus demonstrating the correctness of the
  // sampling method.
  constexpr unsigned int D = TypeParam::value;
  constexpr unsigned int SAMPLE_N = 50000;
  constexpr unsigned int TRIES = 11;
  // We are only approximating the parameters so low precision is expected.
  constexpr double PREC = 0.02;
  for (double scale : SCALES) {
    for (int i = 0; i < TRIES; ++i) {
      const typename GaussianTest<TypeParam>::GaussParams p =
          this->generate_parameters(D, scale);
      Gaussian<D> g(p.mu, p.cov);
      // Draw samples
      const Eigen::MatrixXd samples = g.samples(SAMPLE_N);
      // Compute the mean
      const Eigen::Matrix<double, D, 1> mu_c = mean(samples);
      // Compute the covariance
      const Eigen::Matrix<double, D, D> cov_c = covariance(samples);
      // Compare the computed statistics to the parameters of the distribution
      // being sampled.
      EXPECT_TRUE(mu_c.isApprox(g.mu(), PREC));
      EXPECT_TRUE(cov_c.isApprox(g.cov(), PREC));
    }
  }
}

TYPED_TEST(GaussianTest, SingleSampling) {
  // Gaussian<D>::sample() is simply a convenience overload of samples(1).
  // Given that samples(n) is tested above, this test simply asserts that
  // samples returns a single row matrix (vector) as expected.
  constexpr unsigned int D = TypeParam::value;
  const typename GaussianTest<TypeParam>::GaussParams p =
      this->generate_parameters(D);
  Gaussian<D> g(p.mu, p.cov);
  Eigen::Matrix<double, 1, D> sample;
  EXPECT_NO_THROW({
    // Try to assign the sample to a vector of expected dimensionality.
    sample = g.sample();
  });
}

TYPED_TEST(GaussianTest, SingularMatrix) {
  // The simplest singular matrix is a matrix of zeros.
  constexpr unsigned int D = TypeParam::value;
  const typename GaussianTest<TypeParam>::VectorD mu =
      GaussianTest<TypeParam>::VectorD::Zero();
  const typename GaussianTest<TypeParam>::MatrixDD cov =
      GaussianTest<TypeParam>::MatrixDD::Zero();
  // Build Gaussian of zeros.
  Gaussian<D> g(mu, cov);
  // Sample should be all zeros.
  Eigen::MatrixXd s = g.sample();
  EXPECT_TRUE(s.isZero());
}

template <typename T>
using GaussianDeathTest = GaussianTest<T>;
TYPED_TEST_SUITE(GaussianDeathTest, DimNonTypes);

TYPED_TEST(GaussianDeathTest, NonSymmetricCov) {
  constexpr unsigned int D = TypeParam::value;
  const typename GaussianTest<TypeParam>::VectorD mu =
      GaussianTest<TypeParam>::VectorD::Zero();
  const typename GaussianTest<TypeParam>::MatrixDD cov =
      this->generate_randn_matrix(D, D);
  // Check that random did not accidentally generate a symmetric matrix.
  ASSERT_FALSE(cov.isApprox(cov.transpose()));
  EXPECT_DEATH(
      {
        Gaussian<D> g(mu, cov);
        (void)g;  // Avoid unused variable errors.
      },
      "Covariance matrix must be symmetric");
}

TYPED_TEST(GaussianDeathTest, NegativeEigenvalues) {
  constexpr unsigned int D = TypeParam::value;
  const typename GaussianTest<TypeParam>::VectorD mu =
      GaussianTest<TypeParam>::VectorD::Zero();
  // Negative numbers on the diagonal will result in negative eigenvalues.
  const typename GaussianTest<TypeParam>::MatrixDD cov =
      GaussianTest<TypeParam>::MatrixDD::Identity() * -1;
  EXPECT_DEATH(
      {
        Gaussian<D> g(mu, cov);
        (void)g;  // Avoid unused variable errors.
      },
      "Symmetric PSD matrices have positive eigenvalues");
}

}  // namespace resim::math
