#include "resim/geometry/bounding_box_from_wireframe.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cstdint>
#include <random>

#include "resim/assert/assert.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/se3.hh"

namespace resim::geometry {

using transforms::SE3;
using Vec3 = Eigen::Vector3d;
using Frame = transforms::Frame<transforms::SE3::DIMS>;

// Test that we fail when an empty wireframe is passed in.
TEST(BoundingBoxFromWireframeDeathTest, TestEmptyWireframe) {
  // SETUP
  const Wireframe empty_wireframe{{}, {}};

  // ACTION / VERIFICATION
  EXPECT_THROW(
      { bounding_box_from_wireframe(empty_wireframe); },
      AssertException);
}

// Test that we fail when a wireframe with at least one zero extent is passed
// in.
TEST(BoundingBoxFromWireframeDeathTest, TestFlatWireframe) {
  // SETUP
  // Set up a wireframe with no finite Z extent.
  const Wireframe flat_wireframe{
      {Vec3::UnitX(), -Vec3::UnitX(), Vec3::UnitY(), -Vec3::UnitY()},
      std::vector<Wireframe::Edge>{}};

  // ACTION / VERIFICATION
  EXPECT_THROW(
      { bounding_box_from_wireframe(flat_wireframe); },
      AssertException);
}

// Test that we correctly compute bounding boxes for some random wireframes.
TEST(BoundingBoxFromWireframeTest, TestBoundingBoxComputation) {
  // SETUP
  constexpr uint32_t SEED{325U};
  constexpr double LB = 0.1;
  constexpr double UB = 2.0;
  std::mt19937 rng{SEED};
  std::uniform_real_distribution<double> dist{LB, UB};

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Frame reference_frame{Frame::new_frame()};
    const Frame box_frame{Frame::new_frame()};

    const Vec3 expected_extents{testing::random_vector<Vec3>(rng, dist)};
    const Vec3 expected_translation{testing::random_vector<Vec3>(rng, dist)};

    const std::vector<Vec3> wireframe_points{
        expected_translation + 0.5 * expected_extents.x() * Vec3::UnitX(),
        expected_translation - 0.5 * expected_extents.x() * Vec3::UnitX(),
        expected_translation + 0.5 * expected_extents.y() * Vec3::UnitY(),
        expected_translation - 0.5 * expected_extents.y() * Vec3::UnitY(),
        expected_translation + 0.5 * expected_extents.z() * Vec3::UnitZ(),
        expected_translation - 0.5 * expected_extents.z() * Vec3::UnitZ(),
    };
    const Wireframe wireframe{wireframe_points, {}};

    // ACTION
    const OrientedBox<SE3> bounding_box{
        bounding_box_from_wireframe(wireframe, reference_frame, box_frame)};

    // VERIFICATION
    EXPECT_TRUE(bounding_box.extents().isApprox(expected_extents));
    EXPECT_TRUE(bounding_box.reference_from_box().translation().isApprox(
        expected_translation));
    EXPECT_EQ(bounding_box.reference_from_box().into(), reference_frame);
    EXPECT_EQ(bounding_box.reference_from_box().from(), box_frame);
  }
}

}  // namespace resim::geometry
