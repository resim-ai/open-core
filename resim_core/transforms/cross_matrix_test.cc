#include "resim_core/transforms/cross_matrix.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "resim_core/transforms/liegroup_test_helpers.hh"

namespace resim::transforms {

TEST(CrossMatrixTest, CrossEquivalence) {
  const auto test_vectors = make_test_vectors<Eigen::Vector3d>();
  const Eigen::Vector3d &other = test_vectors.back();
  for (const Eigen::Vector3d &vector : test_vectors) {
    Eigen::Vector3d cross_a = cross_matrix(vector) * other;
    Eigen::Vector3d cross_b = vector.cross(other);
    // Some of our test vectors are near zero.
    EXPECT_TRUE((cross_a - cross_b).isZero());
  }
}

}  // namespace resim::transforms
