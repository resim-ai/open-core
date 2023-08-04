// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/transforms/cross_matrix.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "resim/math/is_approx.hh"
#include "resim/transforms/liegroup_test_helpers.hh"

namespace resim::transforms {

TEST(CrossMatrixTest, CrossEquivalence) {
  const auto test_vectors = make_test_vectors<Eigen::Vector3d>();
  const Eigen::Vector3d &other = test_vectors.back();
  for (const Eigen::Vector3d &vector : test_vectors) {
    Eigen::Vector3d cross_a = cross_matrix(vector) * other;
    Eigen::Vector3d cross_b = vector.cross(other);
    EXPECT_TRUE(math::is_approx(cross_a, cross_b));
  }
}

}  // namespace resim::transforms
