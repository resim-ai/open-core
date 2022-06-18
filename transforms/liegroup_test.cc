#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <vector>

#include "transforms/liegroup_test_helpers.hh"
#include "transforms/se3.hh"
#include "transforms/so3.hh"

namespace resim {
namespace transforms {

// Common tests that apply over all LieGroup classes, for example: SO3, SE3.
// For specialized tests see the test files for the classes themselves, for
// example so3_test.cc
template <typename t>
class LieGroupTests : public ::testing::Test {};

using LieGroupTypes = ::testing::Types<SO3, SE3>;
TYPED_TEST_SUITE(LieGroupTests, LieGroupTypes);

TYPED_TEST(LieGroupTests, inverse_negative_alg_equivalence) {
  for (const typename TypeParam::TangentVector &alg :
       make_test_algebra_elements<TypeParam>()) {
    const TypeParam b_from_a_ref = TypeParam::exp(alg).inverse();
    const TypeParam b_from_a = TypeParam::exp(-alg);
    EXPECT_TRUE(b_from_a_ref.is_approx(b_from_a));
  }
}

TYPED_TEST(LieGroupTests, interp_zero_identity) {
  // Confirm interpolating at zero gives identity.
  constexpr double ZERO = 0;
  for (const TypeParam &a_from_b : make_test_group_elements<TypeParam>()) {
    EXPECT_TRUE(a_from_b.interp(ZERO).is_approx(TypeParam::identity()));
  }
}

TYPED_TEST(LieGroupTests, interp_one_noop) {
  // Confirm interpolating at one is a noop.
  constexpr double ONE = 1.;
  for (const TypeParam &a_from_b : make_test_group_elements<TypeParam>()) {
    EXPECT_TRUE(a_from_b.interp(ONE).is_approx(a_from_b));
  }
}

TYPED_TEST(LieGroupTests, exp_of_zero) {
  TypeParam a_from_a_ref = TypeParam::identity();
  TypeParam a_from_a = TypeParam::exp(TypeParam::TangentVector::Zero());
  EXPECT_TRUE(a_from_a_ref.is_approx(a_from_a));
}

TYPED_TEST(LieGroupTests, log_of_identity) {
  const TypeParam a_from_a = TypeParam::identity();
  // Test log of identity TypeParam is zero.
  EXPECT_TRUE(a_from_a.log().isApprox(TypeParam::TangentVector::Zero()));
}

TYPED_TEST(LieGroupTests, exp_of_log_noop) {
  std::vector<TypeParam> test_elements = make_test_group_elements<TypeParam>();
  // Exp should always be the inverse of log.
  for (const TypeParam &a_from_b : test_elements) {
    const TypeParam a_from_b_ref = TypeParam::exp(a_from_b.log());
    EXPECT_TRUE(a_from_b_ref.is_approx(a_from_b));
  }
}

TYPED_TEST(LieGroupTests, self_adjoint_noop) {
  for (const typename TypeParam::TangentVector &alg :
       make_test_algebra_elements<TypeParam>()) {
    const TypeParam a_from_b = TypeParam::exp(alg);
    const typename TypeParam::TangentVector alg_noop =
        a_from_b.adjoint_times(a_from_b.log());
    EXPECT_TRUE(alg.isApprox(alg_noop));
  }
}

TYPED_TEST(LieGroupTests, composition_by_adjoint) {
  const auto test_elements = make_test_group_elements<TypeParam>();
  TypeParam a_from_b = test_elements.back();
  for (const TypeParam &b_from_c : test_elements) {
    const TypeParam a_from_c_ref = a_from_b * b_from_c;
    const TypeParam a_from_c =
        TypeParam::exp(a_from_b.adjoint_times(b_from_c.log())) * a_from_b;
    EXPECT_TRUE(a_from_c_ref.is_approx(a_from_c));
  }
}

TYPED_TEST(LieGroupTests, floating_point_equality) {
  const auto test_elements = make_test_group_elements<TypeParam>();
  TypeParam a_from_a = test_elements.front();
  for (auto test_elements_it = test_elements.begin() + 1;
       test_elements_it < test_elements.end();
       ++test_elements_it) {
    const TypeParam &a_from_b = *test_elements_it;
    const TypeParam &a_from_b_cp = *test_elements_it;
    EXPECT_TRUE(a_from_b.is_approx(a_from_b_cp));
    // Given that all the test elements are guaranteed to be unique - a
    // constraint that is enforced in the test for the helper lib - then it is
    // reasonable to expect the below to test to return false consistently.
    EXPECT_FALSE(a_from_a.is_approx(a_from_b));
  }
}

}  // namespace transforms
}  // namespace resim
