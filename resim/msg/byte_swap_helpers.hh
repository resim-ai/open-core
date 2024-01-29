// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// byte_swap_helpers.hh
//
// This header contains helper functions used to ensure that we can
// always write and read certain data types to protobuf bytes in
// little endian order, regardless of platform. This is necssary for
// int16, for instance, because protobuf does not have a native int16
// type.

#include <bit>
#include <cstdint>

#include "resim/msg/primitives.pb.h"
#include "resim/utils/inout.hh"

namespace resim::msg {

// Set the data field of Int16 to match an int in little endian
// order (or big endian order if specified).
template <std::endian target_order = std::endian::little>
void set_data(int16_t data, InOut<Int16> msg);

// Set the data field of UInt16 to match an int in little endian
// order (or big endian order if specified).
template <std::endian target_order = std::endian::little>
void set_data(uint16_t data, InOut<UInt16> msg);

// Get the data field of Int16 in little endian order (or big endian
// order if specified).
template <std::endian target_order = std::endian::little>
int16_t data(const Int16 &msg);

// Get the data field of UInt16 in little endian order (or big endian
// order if specified).
template <std::endian target_order = std::endian::little>
uint16_t data(const UInt16 &msg);

}  // namespace resim::msg
