#include "transforms/liegroup_test_helpers.hh"
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

TEST(SO3ConstructorTest, matrix_identity) {
    // Initialize SO3 with an identiy matrix.
    const SO3 from_id_mat(Eigen::Matrix3d::Identity());
    // Initializes to identity
    EXPECT_TRUE(from_id_mat.rotation_matrix().isApprox(
        Eigen::Matrix3d::Identity()));
    // Compate to identity SO3
    const SO3 identity = SO3::identity();
    EXPECT_TRUE(identity.is_approx(from_id_mat));
}

TEST(SO3ExpLog, exp_of_zero) {
    SO3 frame_from_frame = SO3::exp(SO3::TangentVector::Zero());
    EXPECT_TRUE(frame_from_frame.rotation_matrix().isApprox(
        Eigen::Matrix3d::Identity()));
}

TEST(SO3ExpLog, log_of_identity) {
    const SO3 identity = SO3::identity();
    // Test log of identity SO3 is zero.
    EXPECT_TRUE(identity.log().isApprox(SO3::TangentVector::Zero()));
}

TEST(SO3ExpLog, exp_of_log_noop) {
    std::vector<SO3> test_elements = make_test_group_elements<SO3>();
    // Exp should always be the inverse of log.
    for (const SO3 &frame_a_from_frame_b : test_elements) {
        const SO3 frame_a_from_frame_b_expected
            = SO3::exp(frame_a_from_frame_b.log());
        EXPECT_TRUE(frame_a_from_frame_b_expected.is_approx(
            frame_a_from_frame_b));
    }
}
} // namespace transforms
} // namespace resim
