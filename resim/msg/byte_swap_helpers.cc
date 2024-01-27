// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/byte_swap_helpers.hh"

#include <bit>
#include <cstring>
#include <iomanip>
#include <utility>

namespace resim::msg {

void set_data(int16_t data, InOut<Int16> msg) {
  constexpr size_t NUM_BYTES = 2;
  std::string serialized{NUM_BYTES, '\0'};
  auto data_bits = std::bit_cast<std::array<std::byte, NUM_BYTES>>(data);

  static_assert(
      std::endian::native == std::endian::little or
      std::endian::native == std::endian::big);
  if constexpr (std::endian::native == std::endian::big) {
    std::swap(data_bits.at(0), data_bits.at(1));
  }

  std::memcpy(serialized.data(), &data_bits, NUM_BYTES);
  msg->set_data(serialized);
}

void set_data(const uint16_t data, InOut<UInt16> msg) {
  constexpr size_t NUM_BYTES = 2;
  std::string serialized{NUM_BYTES, '\0'};
  auto data_bits = std::bit_cast<std::array<std::byte, NUM_BYTES>>(data);

  static_assert(
      std::endian::native == std::endian::little or
      std::endian::native == std::endian::big);
  if constexpr (std::endian::native == std::endian::big) {
    std::swap(data_bits.at(0), data_bits.at(1));
  }
  std::memcpy(serialized.data(), &data_bits, NUM_BYTES);
  msg->set_data(serialized);
}

int16_t data(const Int16 &msg) {
  constexpr size_t NUM_BYTES = 2;
  std::array<std::byte, NUM_BYTES> data_bits;
  std::memcpy(data_bits.data(), msg.data().data(), NUM_BYTES);

  static_assert(
      std::endian::native == std::endian::little or
      std::endian::native == std::endian::big);
  if constexpr (std::endian::native == std::endian::big) {
    std::swap(data_bits.at(0), data_bits.at(1));
  }
  return std::bit_cast<int16_t>(data_bits);
}

uint16_t data(const UInt16 &msg) {
  constexpr size_t NUM_BYTES = 2;
  std::array<std::byte, NUM_BYTES> data_bits;
  std::memcpy(data_bits.data(), msg.data().data(), NUM_BYTES);

  static_assert(
      std::endian::native == std::endian::little or
      std::endian::native == std::endian::big);
  if constexpr (std::endian::native == std::endian::big) {
    std::swap(data_bits.at(0), data_bits.at(1));
  }
  return std::bit_cast<uint16_t>(data_bits);
}

}  // namespace resim::msg
