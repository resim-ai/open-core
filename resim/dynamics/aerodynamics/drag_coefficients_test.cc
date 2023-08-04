// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/dynamics/aerodynamics/drag_coefficients.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/testing/random_matrix.hh"

namespace resim::dynamics::aerodynamics {

TEST(DragCoefficientsTest, DefaultConstructor) {
  // SETUP
  const DragCoefficients dc{};

  // VERIFICATION
  EXPECT_EQ(dc, Eigen::Vector3d::Zero());
  EXPECT_EQ(dc.lift_coeff(), 0.0);
  EXPECT_EQ(dc.drag_coeff(), 0.0);
  EXPECT_EQ(dc.pitch_coeff(), 0.0);
}
TEST(DragCoefficientsTests, ConstructAndAccess) {
  // SETUP
  constexpr std::size_t SEED = 9381U;
  std::mt19937 rng{SEED};
  const Eigen::Vector3d test_vector{
      testing::random_vector<Eigen::Matrix<double, 3, 1>>(rng)};

  // ACTION
  const DragCoefficients dc{test_vector};
  const DragCoefficients dc2{test_vector.x(), test_vector.y(), test_vector.z()};

  // VERIFICATION
  EXPECT_EQ(dc, test_vector);
  EXPECT_EQ(dc, dc2);
  EXPECT_EQ(dc.lift_coeff(), test_vector.x());
  EXPECT_EQ(dc.drag_coeff(), test_vector.y());
  EXPECT_EQ(dc.pitch_coeff(), test_vector.z());
}

TEST(DragCoefficientsTests, Lerp) {
  // SETUP
  constexpr std::size_t SEED = 4206732U;
  std::mt19937 rng{SEED};
  const Eigen::Vector3d test_vector1{
      testing::random_vector<Eigen::Matrix<double, 3, 1>>(rng)};
  const Eigen::Vector3d test_vector2{
      testing::random_vector<Eigen::Matrix<double, 3, 1>>(rng)};
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  const double test_t = uniform(rng);

  // ACTION
  const DragCoefficients dc1{test_vector1};
  const DragCoefficients dc2{test_vector2};

  const Eigen::Vector3d vector_sum =
      test_t * test_vector1 + (1.0 - test_t) * test_vector2;
  const DragCoefficients dc_sum = test_t * dc1 + dc2 * (1.0 - test_t);

  // VERIFICATION
  EXPECT_EQ(vector_sum, dc_sum);
}

TEST(DragCoefficientsTest, Ostream) {
  // SETUP
  const DragCoefficients dc{-0.1, 0.0, 1.528};

  std::stringstream o;
  o << dc;

  EXPECT_EQ(
      o.str(),
      "DragCoefficients(lift_coeff: -0.1, drag_coeff: 0, pitch_coeff: 1.528)");
}

// The following should *not* compile:

// TEST(DragCoefficientsTest, BadAdd) {
//   // SETUP
//   const DragCoefficients dc{-0.1, 0.0, 1.528};
//   const Eigen::Vector3d vec{0.4, 0.1, 0.7};

//   auto s = dc + vec;
// }
}  // namespace resim::dynamics::aerodynamics
