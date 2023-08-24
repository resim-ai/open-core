// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/math/is_approx.hh"

#include <gtest/gtest.h>

#include "resim/testing/random_matrix.hh"

namespace resim::math {

using Vec3 = Eigen::Vector3d;

// Test that is_approx() works absolutely rather than relatively when small.
TEST(IsApproxTest, TestIsApproxSmall) {
  // SETUP
  constexpr int SEED = 923U;
  std::mt19937 rng{SEED};

  constexpr double MAGNITUDE = 0.1;
  constexpr double TOLERANCE = 1e-5;
  constexpr double EPSILON = 1e-12;

  const Vec3 a{MAGNITUDE * testing::random_matrix<Vec3>(rng).normalized()};
  const Vec3 should_match_a{a + (TOLERANCE - EPSILON) * a.normalized()};
  const Vec3 shouldnt_match_a{a + (TOLERANCE + EPSILON) * a.normalized()};

  // ACTION / VERIFICATION
  EXPECT_TRUE(is_approx(a, should_match_a, TOLERANCE));
  EXPECT_FALSE(is_approx(a, shouldnt_match_a, TOLERANCE));

  EXPECT_TRUE(is_approx(a.norm(), should_match_a.norm(), TOLERANCE));
  EXPECT_FALSE(is_approx(a.norm(), shouldnt_match_a.norm(), TOLERANCE));
}

// Test that is_approx() works relatively rather than absolutely when large.
TEST(IsApproxTest, TestIsApproxLarge) {
  // SETUP
  constexpr int SEED = 923U;
  std::mt19937 rng{SEED};

  constexpr double MAGNITUDE = 1000.0;
  constexpr double TOLERANCE = 1e-5;
  constexpr double EPSILON = 1e-12;

  const Vec3 a{MAGNITUDE * testing::random_matrix<Vec3>(rng).normalized()};
  const Vec3 should_match_a{(1. + (TOLERANCE - EPSILON)) * a};
  const Vec3 shouldnt_match_a{(1. + (TOLERANCE + EPSILON)) * a};

  // ACTION / VERIFICATION
  EXPECT_TRUE(is_approx(a, should_match_a, TOLERANCE));
  EXPECT_FALSE(is_approx(a, shouldnt_match_a, TOLERANCE));

  EXPECT_TRUE(is_approx(a.norm(), should_match_a.norm(), TOLERANCE));
  EXPECT_FALSE(is_approx(a.norm(), shouldnt_match_a.norm(), TOLERANCE));
}

}  // namespace resim::math
