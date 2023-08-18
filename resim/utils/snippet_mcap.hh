// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <mcap/reader.hpp>
#include <mcap/writer.hpp>

#include "resim/time/timestamp.hh"
#include "resim/utils/inout.hh"

namespace resim {

// This function takes all messages (along with their channels and schemas) from
// the interval [start_time, end_time) in the input_mcap and writes them to the
// output_mcap. It does not copy attachements.
// TODO(mikebauer) Support attachements.
// @param[in] start_time - The start of the interval. This end is inclusive.
// @param[in] end_time - The end of the interval. This end is exclusive.
// @param[in] input_mcap - The input mcap to read from.
// @param[out] output_mcap - The output mcap to write to. This should be empty
// to
//                          avoid channel + schema collisions.
void snippet_mcap(
    time::Timestamp start_time,
    time::Timestamp end_time,
    InOut<mcap::McapReader> input_mcap,
    InOut<mcap::McapWriter> output_mcap);

}  // namespace resim
