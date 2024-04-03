// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <gtest/gtest.h>

#include <random>

#include "resim/converter/fuzz_helpers.hh"
#include "resim/utils/inout.hh"

namespace resim::converter {

template <typename T>
class FuzzHelpersTestTemplate : public ::testing::Test {
  static constexpr size_t SEED = 93U;

 public:
  FuzzHelpersTestTemplate() : rng_{SEED} {}

  std::mt19937& rng() { return rng_; };

 private:
  std::mt19937 rng_;
};

TYPED_TEST_SUITE_P(FuzzHelpersTestTemplate);

TYPED_TEST_P(FuzzHelpersTestTemplate, TestRandomElementAndEquality) {
  constexpr int NUM_TESTS = 10;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const TypeParam element_1{
        converter::random_element<TypeParam>(InOut{this->rng()})};
    const TypeParam element_2{
        converter::random_element<TypeParam>(InOut{this->rng()})};

    EXPECT_TRUE(converter::verify_equality(element_1, element_1));
    EXPECT_TRUE(converter::verify_equality(element_2, element_2));

    EXPECT_FALSE(converter::verify_equality(element_1, element_2));
    EXPECT_FALSE(converter::verify_equality(element_2, element_1));
  }
}

REGISTER_TYPED_TEST_SUITE_P(
    FuzzHelpersTestTemplate,
    TestRandomElementAndEquality);

}  // namespace resim::converter
