#include "resim_core/utils/nullable_reference.hh"

#include <gtest/gtest.h>

#include "resim_core/assert/assert.hh"

namespace resim {

TEST(NullableReferenceTest, TestNullableReference) {
  // SETUP
  constexpr int TARGET_VALUE = 4;

  enum class DidSet {
    SET_VALUE = 0,
    NO_SET_VALUE,
  };
  const auto test_function = [&](NullableReference<int> out) -> DidSet {
    if (out) {
      *out = TARGET_VALUE;
      return DidSet::SET_VALUE;
    }
    return DidSet::NO_SET_VALUE;
  };

  // ACTION / VERFICIATION
  EXPECT_EQ(test_function(null_reference<int>), DidSet::NO_SET_VALUE);

  int value_to_set = 0;
  EXPECT_EQ(test_function(NullableReference{value_to_set}), DidSet::SET_VALUE);
  EXPECT_EQ(value_to_set, TARGET_VALUE);
}

TEST(NullableReferenceTest, TestArrowOperator) {
  // SETUP
  constexpr int NEW_VALUE = 4;
  struct TestStruct {
    int value = NEW_VALUE - 1;
  };

  const auto test_function = [&](NullableReference<TestStruct> out) {
    if (out) {
      out->value = NEW_VALUE;
    }
  };

  // ACTION / VERFICIATION
  TestStruct value_to_set;
  test_function(NullableReference{value_to_set});
  EXPECT_EQ(value_to_set.value, NEW_VALUE);
}

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(NullableReferenceDeathTest, TestBadDereference) {
  // SETUP
  constexpr int TARGET_VALUE = 4;
  const auto test_function = [&](NullableReference<int> out) {
    *out = TARGET_VALUE;
  };

  // ACTION / VERFICIATION
  EXPECT_THROW({ test_function(null_reference<int>); }, AssertException);
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim
