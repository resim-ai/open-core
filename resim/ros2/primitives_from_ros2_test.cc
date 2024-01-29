// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/ros2/primitives_from_ros2.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/msg/fuzz_helpers.hh"
#include "resim/msg/primitives.pb.h"
#include "resim/testing/fuzz_helpers.hh"
#include "resim/utils/inout.hh"

namespace resim::ros2 {

using Types = ::testing::Types<
    msg::Bool,
    msg::Byte,
    msg::Char,
    msg::Empty,
    msg::Float32,
    msg::Float64,
    msg::Int16,
    msg::Int32,
    msg::Int64,
    msg::Int8,
    msg::String,
    msg::UInt16,
    msg::UInt32,
    msg::UInt64,
    msg::UInt8>;

template <typename T>
struct PrimitivesFromRos2Test : public ::testing::Test {};

TYPED_TEST_SUITE(PrimitivesFromRos2Test, Types);

TYPED_TEST(PrimitivesFromRos2Test, TestRoundTrip) {
  // SETUP
  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};
  const TypeParam test_element{random_element<TypeParam>(InOut{rng})};

  // ACTION
  const TypeParam round_tripped{
      convert_from_ros2(convert_to_ros2(test_element))};

  // VERIFICATION
  EXPECT_TRUE(verify_equality(test_element, round_tripped));
}

}  // namespace resim::ros2
