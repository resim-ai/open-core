// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/byte_swap_helpers.hh"

#include <gtest/gtest.h>

#include <bit>
#include <cstring>

#include "resim/msg/primitives.pb.h"
#include "resim/utils/inout.hh"

namespace resim::msg {

TEST(ByteSwapHelpersTest, TestSetBytes) {
  // SETUP
  // Little endian
  std::vector<uint8_t> expected_bytes{0x5a, 0xb7};
  constexpr int16_t TEST_INT = -18598;
  constexpr uint16_t TEST_UINT = 46938;

  Int16 int16_msg;
  UInt16 uint16_msg;

  set_data(TEST_INT, InOut{int16_msg});
  set_data(TEST_UINT, InOut{uint16_msg});

  constexpr size_t NUM_BYTES = 2u;
  EXPECT_EQ(
      std::memcmp(int16_msg.data().data(), expected_bytes.data(), NUM_BYTES),
      0u);
  EXPECT_EQ(
      std::memcmp(uint16_msg.data().data(), expected_bytes.data(), NUM_BYTES),
      0u);
}

TEST(ByteSwapHelpersTest, TestSetBytesBigEndian) {
  // SETUP
  // Little endian
  std::vector<uint8_t> expected_bytes{0xb7, 0x5a};
  constexpr int16_t TEST_INT = -18598;
  constexpr uint16_t TEST_UINT = 46938;

  Int16 int16_msg;
  UInt16 uint16_msg;

  set_data<std::endian::big>(TEST_INT, InOut{int16_msg});
  set_data<std::endian::big>(TEST_UINT, InOut{uint16_msg});

  constexpr size_t NUM_BYTES = 2u;
  EXPECT_EQ(
      std::memcmp(int16_msg.data().data(), expected_bytes.data(), NUM_BYTES),
      0u);
  EXPECT_EQ(
      std::memcmp(uint16_msg.data().data(), expected_bytes.data(), NUM_BYTES),
      0u);
}

TEST(ByteSwapHelpersTest, TestGetBytes) {
  // SETUP
  constexpr int16_t TEST_INT = -18598;
  constexpr uint16_t TEST_UINT = 46938;

  Int16 int16_msg;
  UInt16 uint16_msg;

  set_data(TEST_INT, InOut{int16_msg});
  set_data(TEST_UINT, InOut{uint16_msg});

  EXPECT_EQ(TEST_INT, data(int16_msg));
  EXPECT_EQ(TEST_UINT, data(uint16_msg));

  set_data<std::endian::big>(TEST_INT, InOut{int16_msg});
  set_data<std::endian::big>(TEST_UINT, InOut{uint16_msg});

  EXPECT_EQ(TEST_INT, data<std::endian::big>(int16_msg));
  EXPECT_EQ(TEST_UINT, data<std::endian::big>(uint16_msg));
}

}  // namespace resim::msg
