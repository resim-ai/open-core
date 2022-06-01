#include "transforms/liegroup_test_helpers.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>

#include "transforms/so3.hh"

namespace resim {
namespace transforms {

template <typename t>
class LiegroupHelperTests : public ::testing::Test {};

// TODO(simon) add new liegroups to this list when they land (e.g. SE3).
using LiegroupTypes = ::testing::Types<SO3>;
TYPED_TEST_SUITE(LiegroupHelperTests, LiegroupTypes);

TYPED_TEST(LiegroupHelperTests, test_make_algebra_elements) {
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

TYPED_TEST(LiegroupHelperTests, test_make_group_elements) {
  const auto algebra_elements = make_test_algebra_elements<TypeParam>();
  const auto group_elements = make_test_group_elements<TypeParam>();
  // Confirm the two vecors are of the same size.
  EXPECT_EQ(algebra_elements.size(), group_elements.size());
}

}  // namespace transforms
}  // namespace resim
