// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/testing/fuzz_helpers.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/utils/inout.hh"

namespace resim::testing {

namespace some_namespace {

struct TestStruct {
  int val = 0;
};

template <typename Rng>
TestStruct random_element(TypeTag<TestStruct> /*unused*/, InOut<Rng> rng) {
  return TestStruct{
      .val = random_element<int>(rng),
  };
}

bool verify_equality(const TestStruct &a, const TestStruct &b) {
  return a.val == b.val;
}

}  // namespace some_namespace

template <typename T>
struct ContainerFuzzHelpersTest : public ::testing::Test {};

using PayloadTypes = ::testing::Types<double, some_namespace::TestStruct>;

TYPED_TEST_SUITE(ContainerFuzzHelpersTest, PayloadTypes);

TYPED_TEST(ContainerFuzzHelpersTest, TestRandomVector) {
  // SETUP
  std::mt19937 rng;

  const auto test_vec =
      random_element(TypeTag<std::vector<TypeParam>>(), InOut{rng});

  auto test_vec_different_size{test_vec};
  test_vec_different_size.emplace_back(
      resim::random_element<TypeParam>(InOut{rng}));

  auto test_vec_different_value{test_vec};
  test_vec_different_value.front() =
      resim::random_element<TypeParam>(InOut{rng});

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(test_vec, test_vec));

  EXPECT_FALSE(verify_equality(test_vec_different_size, test_vec));
  EXPECT_FALSE(verify_equality(test_vec_different_value, test_vec));

  EXPECT_FALSE(verify_equality(test_vec, test_vec_different_size));
  EXPECT_FALSE(verify_equality(test_vec, test_vec_different_value));
}

}  // namespace resim::testing
