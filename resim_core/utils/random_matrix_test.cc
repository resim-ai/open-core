
#include "resim_core/utils/random_matrix.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <random>

namespace resim::testing {

template <typename T>
class RandomVectorTests : public ::testing::Test {};

using VectorTypes =
    ::testing::Types<Eigen::Vector2d, Eigen::Vector3d, Eigen::Vector4d>;

TYPED_TEST_SUITE(RandomVectorTests, VectorTypes);

// Test that the random_matrix() function returns pseudo-randomly
// generated vectors with independent entries that have the right
// means and variances.
TYPED_TEST(RandomVectorTests, RandomVectorTest) {
  // SETUP
  constexpr double LOWER_BOUND = 0.0;
  constexpr double UPPER_BOUND = 1.0;
  constexpr int NUM_SAMPLES = 100000;

  constexpr size_t ROWS = TypeParam::RowsAtCompileTime;
  using Covariance = Eigen::Matrix<double, ROWS, ROWS>;

  constexpr unsigned SEED = 5301U;
  std::mt19937 rng{SEED};
  std::uniform_real_distribution<double> dist{LOWER_BOUND, UPPER_BOUND};

  // ACTION
  // Streaming covariance computation
  // (https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online)
  TypeParam mean = TypeParam::Zero();
  Covariance covariance = Covariance::Zero();
  for (int idx = 1; idx <= NUM_SAMPLES; ++idx) {
    const TypeParam sample{random_matrix<TypeParam>(rng, dist)};
    const TypeParam residual{sample - mean};
    mean += residual / idx;
    covariance += residual * (sample - mean).transpose();
  }
  covariance /= (NUM_SAMPLES - 1);

  // VERIFICATION
  constexpr double TOLERANCE = 1e-1;
  const TypeParam expected_mean{
      TypeParam::Ones() * 0.5 * (LOWER_BOUND + UPPER_BOUND)};
  EXPECT_TRUE(mean.isApprox(expected_mean, TOLERANCE));

  const Covariance expected_covariance{
      Covariance::Identity() * std::pow(UPPER_BOUND - LOWER_BOUND, 2.0) / 12.0};
  EXPECT_TRUE(covariance.isApprox(expected_covariance, TOLERANCE * TOLERANCE));
}

// Test that the overload with a default distribution behaves as
// expected. We test that the overload is consistent with passing in
// the distribution manually.
TYPED_TEST(RandomVectorTests, RandomVectorOverloadTest) {
  // SETUP
  constexpr unsigned SEED = 9653U;
  constexpr double LOWER_BOUND = -1.0;
  constexpr double UPPER_BOUND = 1.0;
  constexpr int NUM_SAMPLES = 1000;

  std::mt19937 rng_1{SEED};
  std::mt19937 rng_2{SEED};
  std::uniform_real_distribution<double> dist{LOWER_BOUND, UPPER_BOUND};

  // ACTION / VERIFICATION
  for (int ii = 0; ii < NUM_SAMPLES; ++ii) {
    EXPECT_EQ(
        random_matrix<TypeParam>(rng_1, dist),
        random_matrix<TypeParam>(rng_2));
  }
}

}  // namespace resim::testing
