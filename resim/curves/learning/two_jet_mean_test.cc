// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/learning/two_jet_mean.hh"

#include <gtest/gtest.h>

#include <algorithm>
#include <random>

#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/curves/two_jet_test_helpers.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/inout.hh"

namespace resim::curves::learning {

using transforms::SE3;
using TwoJetL = TwoJetL<SE3>;
using TangentVector = optimization::TwoJetTangentVector<SE3>;

// Simple helper to create random sets of TwoJets drawn from a normal
// distribution.
template <typename Rng>
std::vector<TwoJetL> random_samples(
    const int num_samples,
    const TwoJetL &dist_mean,
    const double stddev,
    InOut<Rng> rng) {
  std::vector<TwoJetL> samples;
  samples.reserve(num_samples);

  std::normal_distribution<double> dist{0.0, stddev};
  const auto generator = [&]() -> TwoJetL {
    return optimization::accumulate(
        dist_mean,
        testing::random_matrix<TangentVector>(*rng, dist));
  };

  std::generate_n(std::back_inserter(samples), num_samples, generator);

  return samples;
}

TEST(TwoJetMeanTest, TestTwoJetMean) {
  // SETUP
  constexpr std::size_t NUM_SAMPLES = 50;
  constexpr int SEED = 8932;
  constexpr double STDDEV = 0.1;
  constexpr int MAX_ITERATIONS = NUM_SAMPLES;
  constexpr int NUM_TESTS = 10;
  constexpr double TOLERANCE = 1e-8;
  constexpr double MEAN_TOLERANCE = 5e-2;

  TwoJetTestHelper<TwoJetL> helper;
  std::mt19937 rng{SEED};

  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    auto dist_mean = helper.make_test_two_jet();
    const auto samples =
        random_samples(NUM_SAMPLES, dist_mean, STDDEV, InOut{rng});

    // ACTION
    const auto sample_mean_sv =
        two_jet_mean(samples, TOLERANCE, MAX_ITERATIONS);

    // VERIFICATION
    ASSERT_TRUE(sample_mean_sv.ok());
    const TwoJetL &sample_mean = sample_mean_sv.value();

    EXPECT_TRUE(sample_mean.is_approx(dist_mean, MEAN_TOLERANCE));
  }
}

TEST(TwoJetMeanTest, TestTwoJetMeanSingleSample) {
  // SETUP
  constexpr int SEED = 8932;
  constexpr int MAX_ITERATIONS = 10;
  constexpr int NUM_TESTS = 10;
  constexpr double TOLERANCE = 1e-8;
  constexpr double MEAN_TOLERANCE = 1e-12;

  TwoJetTestHelper<TwoJetL> helper;
  std::mt19937 rng{SEED};

  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    auto dist_mean = helper.make_test_two_jet();
    const std::vector<TwoJetL> samples = {dist_mean};

    // ACTION
    const auto sample_mean_sv =
        two_jet_mean(samples, TOLERANCE, MAX_ITERATIONS);

    // VERIFICATION
    ASSERT_TRUE(sample_mean_sv.ok());
    const TwoJetL &sample_mean = sample_mean_sv.value();

    EXPECT_TRUE(sample_mean.is_approx(dist_mean, MEAN_TOLERANCE));
  }
}

TEST(TwoJetMeanTest, TestTwoJetNoConverge) {
  // SETUP
  constexpr std::size_t NUM_SAMPLES = 50;
  constexpr int SEED = 8932;
  constexpr double STDDEV = 0.1;
  constexpr int MAX_ITERATIONS = 1;
  constexpr double TOLERANCE = 1e-8;

  TwoJetTestHelper<TwoJetL> helper;
  std::mt19937 rng{SEED};
  auto dist_mean = helper.make_test_two_jet();
  const auto samples =
      random_samples(NUM_SAMPLES, dist_mean, STDDEV, InOut{rng});

  // ACTION
  const auto sample_mean_sv = two_jet_mean(samples, TOLERANCE, MAX_ITERATIONS);

  // VERIFICATION
  // Shouldn't converge because MAX_ITERATIONS is 1
  EXPECT_FALSE(sample_mean_sv.ok());
}

}  // namespace resim::curves::learning
