#include "resim_core/visualization/view_server/view_server_test_helper.hh"

#include <gtest/gtest.h>

#include "resim_core/assert/assert.hh"
#include "resim_core/curves/d_curve.hh"
#include "resim_core/curves/d_curve_test_helpers.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::visualization::view_server {

namespace {
using transforms::FSE3;
using transforms::SE3;
using transforms::SO3;
}  // namespace

template <typename Viewable>
class ViewServerHelperTests : public ::testing::Test {};

using ViewableTypes = ::testing::Types<
    SE3,
    SO3,
    FSE3,
    curves::DCurve<SE3>,
    curves::DCurve<FSE3>,
    curves::TCurve<FSE3>>;

TYPED_TEST_SUITE(ViewServerHelperTests, ViewableTypes);

TYPED_TEST(ViewServerHelperTests, ReturnCounts) {
  const std::vector<TypeParam> default_results =
      generate_payload_type<TypeParam>();
  EXPECT_EQ(default_results.size(), detail::MIN_TEST_ELEMENTS);

  constexpr unsigned LRG_COUNT = 101;
  const auto large_results = generate_payload_type<TypeParam>(LRG_COUNT);
  EXPECT_EQ(large_results.size(), LRG_COUNT);
}

template <typename T>
using ViewServerHelperDeathTests = ViewServerHelperTests<T>;
TYPED_TEST_SUITE(ViewServerHelperDeathTests, ViewableTypes);

TYPED_TEST(ViewServerHelperDeathTests, TooFewElementsRequested) {
  constexpr unsigned TOO_FEW = detail::MIN_TEST_ELEMENTS - 1;
  EXPECT_THROW(
      {
        const auto test_vec = generate_payload_type<TypeParam>(TOO_FEW);
        (void)test_vec;
      },
      AssertException);
}

}  // namespace resim::visualization::view_server
