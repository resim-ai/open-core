#include "transforms/liegroup_test_helpers.hh"
#include "transforms/so3.hh"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <vector>


namespace resim {
namespace transforms {

// Common tests that apply over all Liegroup classes, for example: SO3, SE3.
// For specialized tests see the test files for the classes themselves, for
// example so3_test.cc
template <typename t>
class LiegroupTests : public ::testing::Test {};

// TODO(simon) add new liegroups to this list when they land (e.g. SE3).
using LiegroupTypes = ::testing::Types<SO3>;
TYPED_TEST_SUITE(LiegroupTests, LiegroupTypes);

TYPED_TEST(LiegroupTests, inverse_negative_alg_equivalence) {
    for (const typename TypeParam::TangentVector &alg 
            : make_test_algebra_elements<TypeParam>()) {
        const TypeParam b_from_a_ref = TypeParam::exp(alg).inverse();
        const TypeParam b_from_a = TypeParam::exp(-alg);
        EXPECT_TRUE(b_from_a_ref.is_approx(b_from_a));
    }
}

TYPED_TEST(LiegroupTests, interp_zero_identity) {
    // Confirm interpolating at zero gives identity.
    constexpr double ZERO = 0;
    for (const TypeParam &a_from_b : make_test_group_elements<TypeParam>()) {
        EXPECT_TRUE(a_from_b.interp(ZERO).is_approx(TypeParam::identity()));
    }
}

TYPED_TEST(LiegroupTests, interp_one_noop) {
    // Confirm interpolating at one is a noop.
    constexpr double ONE = 1.;
    for (const TypeParam &a_from_b : make_test_group_elements<TypeParam>()) {
        EXPECT_TRUE(a_from_b.interp(ONE).is_approx(a_from_b));
    }
}

TYPED_TEST(LiegroupTests, exp_of_zero) {
    TypeParam a_from_a_ref = TypeParam::identity();
    TypeParam a_from_a = TypeParam::exp(TypeParam::TangentVector::Zero());
    EXPECT_TRUE(a_from_a_ref.is_approx(a_from_a));
}

TYPED_TEST(LiegroupTests, log_of_identity) {
    const TypeParam a_from_a = TypeParam::identity();
    // Test log of identity TypeParam is zero.
    EXPECT_TRUE(a_from_a.log().isApprox(TypeParam::TangentVector::Zero()));
}

TYPED_TEST(LiegroupTests, exp_of_log_noop) {
    std::vector<TypeParam> test_elements 
        = make_test_group_elements<TypeParam>();
    // Exp should always be the inverse of log.
    for (const TypeParam &a_from_b : test_elements) {
        const TypeParam a_from_b_ref = TypeParam::exp(a_from_b.log());
        EXPECT_TRUE(a_from_b_ref.is_approx(a_from_b));
    }
}

TYPED_TEST(LiegroupTests, self_adjoint_noop) {
    for (const typename TypeParam::TangentVector &alg 
            : make_test_algebra_elements<TypeParam>()) {
        const TypeParam a_from_b = TypeParam::exp(alg);
        const typename TypeParam::TangentVector alg_noop 
            = a_from_b.adjoint_times(a_from_b.log());
        EXPECT_TRUE(alg.isApprox(alg_noop));
    }
}

TYPED_TEST(LiegroupTests, composition_by_adjoint) {
    const auto test_elements = make_test_group_elements<TypeParam>();
    TypeParam a_from_b = test_elements.front();
    for (const TypeParam &b_from_c : test_elements) {
        const TypeParam a_from_c_ref = a_from_b * b_from_c;
        const TypeParam a_from_c 
            = TypeParam::exp(a_from_b.adjoint_times(b_from_c.log()))
            * a_from_b;
        EXPECT_TRUE(a_from_c_ref.is_approx(a_from_c));
        a_from_b = b_from_c;
    }
}

} // transforms
} // resim
