// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cstdint>

#include "resim/msg/primitives.pb.h"
#include "resim/utils/inout.hh"

namespace resim::msg {

void set_data(int16_t data, InOut<Int16> msg);

void set_data(uint16_t data, InOut<UInt16> msg);

int16_t data(const Int16 &msg);

uint16_t data(const UInt16 &msg);

}  // namespace resim::msg
