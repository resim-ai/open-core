#include "transforms/liegroup_test_helpers.hh"
#include "transforms/so3.hh"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <vector>


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
    // Initialize SO3 with an identity matrix.
    const SO3 from_id_mat(Eigen::Matrix3d::Identity());
    // Initializes to identity
    EXPECT_TRUE(from_id_mat.rotation_matrix().isApprox(
        Eigen::Matrix3d::Identity()));
    // Compare to identity SO3
    const SO3 identity = SO3::identity();
    EXPECT_TRUE(identity.is_approx(from_id_mat));
}

TEST(SO3ConstructorTest, quaternion) {
    constexpr int TRIES = 7;
    constexpr unsigned int SEED = 42;
    // Make a deterministic seed.
    srand(SEED);
    // Build a vector of random Quaternions.
    std::vector<Eigen::Quaterniond> quats;
    for (int i = 0; i < TRIES; ++i) {
        quats.push_back(Eigen::Quaterniond::UnitRandom());
    }
    srand(1); // Reset the seed
    // Create an arbitrary three vector
    const Eigen::Vector3d three_vec = Eigen::Vector3d::Random();
    for (const Eigen::Quaterniond &a_from_b_quat : quats) {
        // Build an SO3 from the quaternion
        const SO3 a_from_b_so3(a_from_b_quat);
        // Apply the action of the Quaternion to the test vector.
        const Eigen::Vector3d vec_quat = a_from_b_quat * three_vec;
        // Apply the action of the SO3 to the test vector.
        const Eigen::Vector3d vec_so3 = a_from_b_so3 * three_vec;
        // Action should be identical
        EXPECT_TRUE(vec_quat.isApprox(vec_so3));
    }
}

TEST(SO3Inverse, compare_to_matrix_inverse) {
    const auto test_so3s = make_test_group_elements<SO3>();
    for (const SO3 &a_from_b: test_so3s) {
        const Eigen::Matrix3d a_from_b_mat 
            = a_from_b.rotation_matrix();
        const SO3 b_from_a = SO3(a_from_b_mat.inverse());
        EXPECT_TRUE(a_from_b.inverse().is_approx(b_from_a));
    }
}

TEST(SO3Inverse, inverse_negative_alg_equivalence) {
    // TODO(simon) this test could be templated for all Liegroups.
    for (const SO3::TangentVector &alg : make_test_algebra_elements<SO3>()) {
        const SO3 b_from_a_ref = SO3::exp(alg).inverse();
        const SO3 b_from_a = SO3::exp(-alg);
        EXPECT_TRUE(b_from_a_ref.is_approx(b_from_a));
    }
}

TEST(SO3Interp, interp_zero_identity) {
    // TODO(simon) this test could be templated for all Liegroups.
    // Confirm interpolating at zero gives identity.
    constexpr double ZERO = 0;
    for (const SO3 &a_from_b : make_test_group_elements<SO3>()) {
        EXPECT_TRUE(a_from_b.interp(ZERO).is_approx(SO3::identity()));
    }
}

TEST(SO3Interp, interp_one_noop) {
    // TODO(simon) this test could be templated for all Liegroups.
    // Confirm interpolating at one is a noop.
    constexpr double ONE = 1.;
    for (const SO3 &a_from_b : make_test_group_elements<SO3>()) {
        EXPECT_TRUE(a_from_b.interp(ONE).is_approx(a_from_b));
    }
}

TEST(SO3Interp, interp_zero_halfway) {
    constexpr double ANGLE = M_PI / 4.;
    constexpr double HALFWAY = 0.5;
    for (const double angle : {ANGLE, -ANGLE}) {
        const SO3 a_from_b(Eigen::AngleAxisd(
            angle, 
            Eigen::Vector3d::UnitZ()));
        // Create a renamed reference to make explicit that the rotation
        // is to be applied twice sequentially.
        const SO3 &b_from_c = a_from_b;
        const SO3 a_from_c = a_from_b * b_from_c;
        EXPECT_TRUE(a_from_c.interp(HALFWAY).is_approx(a_from_b));
    }
}

TEST(SO3Interp, extrap_double) {
    constexpr double ANGLE = M_PI / 4.;
    constexpr double DOUBLE = 2.;
    for (const double angle : {ANGLE, -ANGLE}) {
        const SO3 a_from_b(Eigen::AngleAxisd(
            angle, 
            Eigen::Vector3d::UnitZ()));
        // Create a renamed reference to make explicit that the rotation
        // is to be applied twice sequentially.
        const SO3 &b_from_c = a_from_b;
        const SO3 a_from_c = a_from_b * b_from_c;
        EXPECT_TRUE(a_from_b.interp(DOUBLE).is_approx(a_from_c));
    }
}

TEST(SO3ExpLog, exp_of_zero) {
    // TODO(simon) this test could be templated for all Liegroups.
    SO3 frame_from_frame = SO3::exp(SO3::TangentVector::Zero());
    EXPECT_TRUE(frame_from_frame.rotation_matrix().isApprox(
        Eigen::Matrix3d::Identity()));
}

TEST(SO3ExpLog, log_of_identity) {
    // TODO(simon) this test could be templated for all Liegroups.
    const SO3 identity = SO3::identity();
    // Test log of identity SO3 is zero.
    EXPECT_TRUE(identity.log().isApprox(SO3::TangentVector::Zero()));
}

TEST(SO3ExpLog, exp_of_log_noop) {
    // TODO(simon) this test could be templated for all Liegroups.
    std::vector<SO3> test_elements = make_test_group_elements<SO3>();
    // Exp should always be the inverse of log.
    for (const SO3 &a_from_b : test_elements) {
        const SO3 a_from_b_expected
            = SO3::exp(a_from_b.log());
        EXPECT_TRUE(a_from_b_expected.is_approx(
            a_from_b));
    }
}

TEST(SO3Adjoint, self_adjoint_noop) {
    // TODO(simon) this test could be templated for all Liegroups.
    for (const SO3::TangentVector &alg : make_test_algebra_elements<SO3>()) {
        const SO3 a_from_b = SO3::exp(alg);
        const SO3::TangentVector alg_noop 
            = a_from_b.adjoint_times(a_from_b.log());
        EXPECT_TRUE(alg.isApprox(alg_noop));
    }
}

TEST(SO3Adjoint, composition_by_adjoint) {
    // TODO(simon) this test could be templated for all Liegroups.
    const auto test_so3s = make_test_group_elements<SO3>();
    SO3 a_from_b = test_so3s.front();
    for (const SO3 &b_from_c : test_so3s) {
        const SO3 a_from_c_ref = a_from_b * b_from_c;
        const SO3 a_from_c 
            = SO3::exp(a_from_b.adjoint_times(b_from_c.log()))
            * a_from_b;
        EXPECT_TRUE(a_from_c_ref.is_approx(a_from_c));
        a_from_b = b_from_c;
    }
}

} // namespace transforms
} // namespace resim
