#include "resim_core/transforms/liegroup_test_helpers.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>

#include "resim_core/assert/assert.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::transforms {

template <typename t>
class LieGroupHelperTests : public ::testing::Test {};

using LieGroupTypes = ::testing::Types<SO3, SE3, FSO3, FSE3>;
TYPED_TEST_SUITE(LieGroupHelperTests, LieGroupTypes);

TYPED_TEST(LieGroupHelperTests, ReturnCounts) {
  const auto test_min_vec =
      make_test_vectors<typename TypeParam::TangentVector>();
  EXPECT_EQ(test_min_vec.size(), detail::MIN_TEST_ELEMENTS);
  const auto test_min_alg = make_test_algebra_elements<TypeParam>();
  EXPECT_EQ(test_min_alg.size(), detail::MIN_TEST_ELEMENTS);
  const auto test_min_grp = make_test_group_elements<TypeParam>();
  EXPECT_EQ(test_min_grp.size(), detail::MIN_TEST_ELEMENTS);

  constexpr unsigned LRG_COUNT = 101;
  const auto test_lrg_vec =
      make_test_vectors<typename TypeParam::TangentVector>(LRG_COUNT);
  EXPECT_EQ(test_lrg_vec.size(), LRG_COUNT);
  const auto test_lrg_alg = make_test_algebra_elements<TypeParam>(LRG_COUNT);
  EXPECT_EQ(test_lrg_alg.size(), LRG_COUNT);
  const auto test_lrg_grp = make_test_group_elements<TypeParam>(LRG_COUNT);
  EXPECT_EQ(test_lrg_grp.size(), LRG_COUNT);
}

TYPED_TEST(LieGroupHelperTests, TestMakeAlgebraElements) {
  // Build the vector of test elements
  std::vector<typename TypeParam::TangentVector> test_elements =
      make_test_algebra_elements<TypeParam>();

  // Confirm the vecor is not empty
  EXPECT_FALSE(test_elements.empty());

  // Confirm that all elements are unique
  // First sort
  std::sort(
      test_elements.begin(),
      test_elements.end(),
      [](const typename TypeParam::TangentVector &a,
         const typename TypeParam::TangentVector &b) -> bool {
        //
        return a.norm() > b.norm();
      });

  // Then find adjacent duplicates (or not)
  const auto last = std::adjacent_find(
      test_elements.begin(),
      test_elements.end(),
      [](const typename TypeParam::TangentVector &a,
         const typename TypeParam::TangentVector &b) -> bool {
        //
        return a.isApprox(b);
      });
  EXPECT_EQ(last, test_elements.end());
}

TYPED_TEST(LieGroupHelperTests, TestMakeGroupElements) {
  const auto algebra_elements = make_test_algebra_elements<TypeParam>();
  const auto group_elements = make_test_group_elements<TypeParam>();
  // Confirm the two vecors are of the same size.
  EXPECT_EQ(algebra_elements.size(), group_elements.size());
}

template <typename T>
using LieGroupHelperDeathTests = LieGroupHelperTests<T>;
TYPED_TEST_SUITE(LieGroupHelperDeathTests, LieGroupTypes);

TYPED_TEST(LieGroupHelperDeathTests, TooFewElementsRequested) {
  constexpr unsigned TOO_FEW = detail::MIN_TEST_ELEMENTS - 1;
  EXPECT_THROW(
      {
        const auto test_vec =
            make_test_vectors<typename TypeParam::TangentVector>(TOO_FEW);
        (void)test_vec;
      },
      AssertException);
  EXPECT_THROW(
      {
        const auto test_alg = make_test_algebra_elements<TypeParam>(TOO_FEW);
        (void)test_alg;
      },
      AssertException);
  EXPECT_THROW(
      {
        const auto test_grp = make_test_group_elements<TypeParam>(TOO_FEW);
        (void)test_grp;
      },
      AssertException);
}

}  // namespace resim::transforms
