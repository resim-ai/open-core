
#include "resim_core/geometry/oriented_box.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::geometry {

using transforms::LieGroupType;

template <LieGroupType T>
class OrientedBoxTest : public ::testing::Test {};

using GroupTypes = ::testing::Types<transforms::SE3, transforms::FSE3>;

TYPED_TEST_SUITE(OrientedBoxTest, GroupTypes);

// Test the getters
TYPED_TEST(OrientedBoxTest, TestGetters) {
  // SETUP
  using TangentVector = typename TypeParam::TangentVector;
  const TypeParam reference_from_box{TypeParam::exp(
      (TangentVector() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished())};
  const Eigen::Vector3d extents{0.3, 0.4, 0.5};
  const OrientedBox box{reference_from_box, extents};

  // ACTION / VERIFICATION
  EXPECT_TRUE(reference_from_box.is_approx(box.reference_from_box()));
  EXPECT_EQ(extents, box.extents());
}

// Test that we correctly detect negative extents.
TYPED_TEST(OrientedBoxTest, TestNegativeExtents) {
  // SETUP
  using TangentVector = typename TypeParam::TangentVector;
  const TypeParam reference_from_box{TypeParam::exp(
      (TangentVector() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished())};
  const Eigen::Vector3d extents{-0.3, 0.4, 0.5};

  // ACTION / VERIFICATION
  EXPECT_DEATH(
      OrientedBox(reference_from_box, extents),
      "Negative extent detected!");

  OrientedBox box{reference_from_box, extents.cwiseAbs()};
  EXPECT_DEATH(box.set_extents(extents), "Negative extent detected!");
}

// Test the setters
TYPED_TEST(OrientedBoxTest, TestSetters) {
  // SETUP
  using TangentVector = typename TypeParam::TangentVector;
  const TypeParam reference_from_box{TypeParam::exp(
      (TangentVector() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished())};
  const Eigen::Vector3d extents{0.3, 0.4, 0.5};
  OrientedBox box{reference_from_box, extents};

  const TypeParam new_reference_from_box{TypeParam::exp(
      (TangentVector() << -1.0, 2.0, -3.0, 4.0, -5.0, 6.0).finished())};
  const Eigen::Vector3d new_extents{2.0 * extents};

  // VERIFICATION
  EXPECT_FALSE(new_reference_from_box.is_approx(box.reference_from_box()));
  EXPECT_NE(new_extents, box.extents());

  // ACTION
  box.set_reference_from_box(new_reference_from_box);
  box.set_extents(new_extents);

  // VERIFICATION
  EXPECT_TRUE(new_reference_from_box.is_approx(box.reference_from_box()));
  EXPECT_EQ(new_extents, box.extents());
}
}  // namespace resim::geometry
