#include "transforms/so3.hh"
#include <gtest/gtest.h>
#include <Eigen/Dense>


namespace resim {
namespace transforms {

TEST(SO3ConstructorTest, angle_axis_zero) {
    // Initialize SO3 with a zero angle angle_axis argument. 
    constexpr double ZERO_ANGLE = 0;
    const SO3 no_rotation(Eigen::AngleAxisd(
        ZERO_ANGLE,
        Eigen::Vector3d::UnitX()));
    // Verify that the rotation matrix is identity.
    EXPECT_TRUE(no_rotation.rotation_matrix().isApprox(
        Eigen::Matrix3d::Identity()));
}

TEST(SO3ConstructorTest, angle_axis_arbitray) {
    // Initialize SO3 with a arbitrary angle angle_axis argument.
    constexpr double ANGLE = 0.732;
    const Eigen::AngleAxisd rotation_aa(ANGLE, Eigen::Vector3d::UnitY());
    const SO3 rotation_so3(rotation_aa);
    // Verify that rotation matrices of axis angle and so3 agree.
    EXPECT_TRUE(rotation_so3.rotation_matrix().isApprox(
        rotation_aa.toRotationMatrix()));
}

} // namespace transforms
} // namespace resim
