// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/utils/type.hh"

#include <gtest/gtest.h>

namespace resim {
namespace test_namespace {

struct TestStruct;

// We don't want this overload to be chosen. It would be chosen if
// Type<T> were not a dependent type. We therefore fail if this overload is
// chosen.
template <typename T>
void test_function(T /* unused */) {
  EXPECT_TRUE(false);
}

}  // namespace test_namespace

// This is the overload we want.
template <typename T>
void test_function(BasicType<T> /* unused */) {
  EXPECT_TRUE(true);
}

// Test that ADL does not consider the overload of test_function that we don't
// want (see above).
TEST(TypeTest, TestADLDisabled) {
  // ACTION / VERIFICATION
  test_function(TypeC<test_namespace::TestStruct>);
}

// Test passing a type into an auto-type-deduced lambda. This is included
// because it is a somewhat common idiom since templates can't help you pass
// type information into lambdas.
TEST(TypeTest, TestPassTypeIntoLambda) {
  // SETUP
  // A simple test lambda which returns an object of the given type (passed in
  // via a Type object).
  const auto test_lambda = [](auto type_arg) ->
      typename decltype(type_arg)::type {
        using ResultType = typename decltype(type_arg)::type;
        // Use ResultType for something
        // ...
        return std::declval<ResultType>();
      };

  // ACTION / VERIFICATION (compile time)
  static_assert(std::is_same_v<decltype(test_lambda(TypeC<int>)), int>);
  static_assert(std::is_same_v<decltype(test_lambda(TypeC<double>)), double>);
  static_assert(std::is_same_v<decltype(test_lambda(TypeC<char>)), char>);
  static_assert(std::is_same_v<
                decltype(test_lambda(TypeC<test_namespace::TestStruct>)),
                test_namespace::TestStruct>);
}

}  // namespace resim
