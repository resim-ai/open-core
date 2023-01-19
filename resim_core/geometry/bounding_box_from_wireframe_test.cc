#include "resim_core/geometry/bounding_box_from_wireframe.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cstdint>
#include <random>

#include "resim_core/testing/random_matrix.hh"
#include "resim_core/transforms/framed_group.hh"

namespace resim::geometry {

using transforms::FSE3;
using Vec3 = Eigen::Vector3d;
using Frame3 = transforms::Frame<3>;

// Test that we fail when an empty wireframe is passed in.
TEST(BoundingBoxFromWireframeDeathTest, TestEmptyWireframe) {
  // SETUP
  const Wireframe empty_wireframe{{}, {}};

  // ACTION / VERIFICATION
  EXPECT_DEATH(
      { bounding_box_from_wireframe(empty_wireframe); },
      "Bounding box can't be found for empty or size one wireframe!");
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
  EXPECT_DEATH(
      { bounding_box_from_wireframe(flat_wireframe); },
      "Wireframe has at least one zero extent!");
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
    const Frame3 reference_frame{Frame3::new_frame()};
    const Frame3 box_frame{Frame3::new_frame()};

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
    const OrientedBox<FSE3> bounding_box{
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
