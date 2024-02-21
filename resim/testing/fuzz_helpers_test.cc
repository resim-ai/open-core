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
struct ContainerFuzzHelpersTest : public ::testing::Test {
  static constexpr size_t SEED = 82U;
  std::mt19937 rng{SEED};
};

using PayloadTypes = ::testing::Types<double, some_namespace::TestStruct>;

TYPED_TEST_SUITE(ContainerFuzzHelpersTest, PayloadTypes);

// Tests that we generate distinct random vectors with random_element.
TYPED_TEST(ContainerFuzzHelpersTest, TestVectorRandom) {
  // SETUP / ACTION
  // Generate a vector of vectors which should each be
  // distinct if we're generating new wones each time.
  const std::vector<std::vector<TypeParam>> random_vectors{
      random_element<std::vector<std::vector<TypeParam>>>(InOut{this->rng})};

  // VERIFICATION
  for (auto it_1 = random_vectors.cbegin(); it_1 != random_vectors.cend();
       ++it_1) {
    for (auto it_2 = std::next(it_1); it_2 != random_vectors.cend(); ++it_2) {
      EXPECT_FALSE(verify_equality(*it_1, *it_2));
    }
  }
}

TYPED_TEST(ContainerFuzzHelpersTest, TestVectorEquality) {
  // SETUP
  const auto test_vec =
      random_element(TypeTag<std::vector<TypeParam>>(), InOut{this->rng});

  auto test_vec_different_size{test_vec};
  test_vec_different_size.emplace_back(
      resim::random_element<TypeParam>(InOut{this->rng}));

  auto test_vec_different_value{test_vec};
  test_vec_different_value.front() =
      resim::random_element<TypeParam>(InOut{this->rng});

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(test_vec, test_vec));

  EXPECT_FALSE(verify_equality(test_vec_different_size, test_vec));
  EXPECT_FALSE(verify_equality(test_vec_different_value, test_vec));

  EXPECT_FALSE(verify_equality(test_vec, test_vec_different_size));
  EXPECT_FALSE(verify_equality(test_vec, test_vec_different_value));
}

}  // namespace resim::testing
