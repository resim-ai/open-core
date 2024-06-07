// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/transforms/proto/fuzz_helpers.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/converter/fuzz_helpers.hh"
#include "resim/utils/inout.hh"

namespace resim::transforms::proto {

TEST(FuzzHelpersTest, TestSE3Equal) {
  // SETUP
  constexpr std::size_t SEED = 3481U;
  std::mt19937 rng{SEED};

  const SE3 se3{converter::random_element<SE3>(InOut{rng})};
  const SE3 different_se3{converter::random_element<SE3>(InOut{rng})};

  // ACTION / VERIFICATION
  EXPECT_TRUE(converter::verify_equality(se3, se3));
  EXPECT_FALSE(converter::verify_equality(se3, different_se3));
  EXPECT_FALSE(converter::verify_equality(different_se3, se3));
}

TEST(FuzzHelpersTest, TestFramedVector3Equal) {
  // SETUP
  constexpr std::size_t SEED = 372736U;
  std::mt19937 rng{SEED};

  const FramedVector_3 fv3{
      converter::random_element<FramedVector_3>(InOut{rng})};
  const FramedVector_3 different_fv3{
      converter::random_element<FramedVector_3>(InOut{rng})};

  // ACTION / VERIFICATION
  EXPECT_TRUE(converter::verify_equality(fv3, fv3));
  EXPECT_FALSE(converter::verify_equality(fv3, different_fv3));
  EXPECT_FALSE(converter::verify_equality(different_fv3, fv3));
}

}  // namespace resim::transforms::proto
