#include "transforms/se3.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "transforms/liegroup_test_helpers.hh"

namespace resim {
namespace transforms {

// Test that an SE3 objects has the expected rotation and translation
// components.
// [param] se3 - the SE3 object to be tested.
// [param] rotation - SO3 representing the expected rotation.
// [param] translation - Vector representing the expected translation.
void check_se3_rotation_and_translation(
    const SE3 &se3,
    const SO3 &rotation,
    const Eigen::Vector3d translation
) {
  //
  EXPECT_TRUE(se3.rotation().is_approx(rotation));
  EXPECT_TRUE(se3.translation().isApprox(translation));
}

TEST(SE3ConstructorTest, rotation_only) {
  for (const SO3 &a_from_b_rot : make_test_group_elements<SO3>()) {
    const SE3 a_from_b(a_from_b_rot);
    check_se3_rotation_and_translation(
        a_from_b,
        a_from_b_rot,
        Eigen::Vector3d::Zero()
    );
  }
}

TEST(SE3ConstructorTest, translation_only) {
  for (const Eigen::Vector3d &a_from_b_trans :
       make_test_vectors<Eigen::Vector3d>()) {
    const SE3 a_from_b(a_from_b_trans);
    check_se3_rotation_and_translation(
        a_from_b,
        SO3::identity(),
        a_from_b_trans
    );
  }
}

TEST(SE3ConstructorTest, rotation_and_translation) {
  const auto trans_elements = make_test_vectors<Eigen::Vector3d>();
  const auto rot_elements = make_test_group_elements<SO3>();
  const unsigned int element_count = trans_elements.size();
  EXPECT_EQ(element_count, rot_elements.size());
  for (int i = 0; i < element_count; ++i) {
    const SE3 a_from_b(rot_elements.at(i), trans_elements.at(i));
    check_se3_rotation_and_translation(
        a_from_b,
        rot_elements.at(i),
        trans_elements.at(i)
    );
  }
}

}  // namespace transforms
}  // namespace resim
