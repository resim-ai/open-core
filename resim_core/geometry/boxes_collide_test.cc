#include "resim_core/geometry/boxes_collide.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_group_concept.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::geometry {

using Vec3 = Eigen::Vector3d;
using transforms::FramedGroupType;
using transforms::FSE3;
using transforms::LieGroupType;
using transforms::SE3;

template <LieGroupType T>
class BoxesCollideTest : public ::testing::Test {};

using GroupTypes = ::testing::Types<SE3, FSE3>;

TYPED_TEST_SUITE(BoxesCollideTest, GroupTypes);

TYPED_TEST(BoxesCollideTest, TestBoxCollisions) {
  // SETUP
  const Vec3 box_a_extents{Vec3::Ones()};
  const Vec3 box_b_extents{2.0 * box_a_extents};
  constexpr double MIN_X = -5.0;
  constexpr double MAX_X = 5.0;
  constexpr int NUM_POINTS = 100;

  const OrientedBox<TypeParam> box_a{TypeParam::identity(), Vec3::Ones()};
  OrientedBox<TypeParam> box_b{TypeParam::identity(), Vec3::Ones()};

  for (int ii = 0; ii < NUM_POINTS; ++ii) {
    const double fraction = static_cast<double>(ii) / (NUM_POINTS - 1);
    const double x = fraction * MAX_X * (1 - fraction) * MIN_X;
    TypeParam reference_from_b{SE3{x * Vec3::UnitX()}};
    if constexpr (FramedGroupType<TypeParam>) {
      reference_from_b.set_into(box_a.reference_from_box().into());
    }
    box_b.set_reference_from_box(reference_from_b);

    // ACTION
    const bool result = boxes_collide(box_a, box_b);

    // VERIFICATION
    const bool expected_result =
        std::fabs(x) < (box_a.extents().x() + box_b.extents().x()) / 2.0;
    EXPECT_EQ(result, expected_result);
  }
}

// Test that we fail on bad frame pairs
// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(BoxesCollideTest, TestFailOnBadFrame) {
  if constexpr (FramedGroupType<TypeParam>) {
    const OrientedBox<TypeParam> box_a{TypeParam::identity(), Vec3::Ones()};
    OrientedBox<TypeParam> box_b{TypeParam::identity(), Vec3::Ones()};
    EXPECT_DEATH(boxes_collide(box_a, box_b), "Box frames don't match!");
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim::geometry
