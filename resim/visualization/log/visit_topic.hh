
#pragma once

#include <functional>
#include <mcap/reader.hpp>
#include <string_view>

#include "resim/utils/inout.hh"

namespace resim::visualization::log {

// A simple function which calls the given visitor on all messages (represented
// by mcap::MessageViews) matching the specified topic in the given log reader.
// @param[in] topic - The topic we want to read from.
// @param[in] reader - The reader to read from. Passed as an inout because
//                     reading is non-const.
// @param[in] visitor - The visitor to call on each message.
void visit_topic(
    std::string_view topic,
    InOut<mcap::McapReader> reader,
    const std::function<void(const mcap::MessageView &)> &visitor);

}  // namespace resim::visualization::log
