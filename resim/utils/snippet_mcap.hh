

#pragma once

#include <mcap/reader.hpp>
#include <mcap/writer.hpp>

#include "resim/time/timestamp.hh"
#include "resim/utils/inout.hh"

namespace resim {

void snippet_mcap(
    time::Timestamp start_time,
    time::Timestamp end_time,
    InOut<mcap::McapReader> input_mcap,
    InOut<mcap::McapWriter> output_mcap);

}
