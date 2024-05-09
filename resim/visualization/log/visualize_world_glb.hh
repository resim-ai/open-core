
#pragma once

#include <filesystem>
#include <istream>
#include <string>

#include "resim/time/timestamp.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/mcap_logger.hh"

namespace resim::visualization::log {

// Given the path of a glb binary file representing the geometry of
// the world frame, log it at the given time to the given channel on
// the given logger. The file is read and the bytes written into the
// data field of a foxglove model primitive. The world file is assumed
// to be attached to the scene frame with no transforms or scaling.
// @param[in] glb_stream - An istream (e.g. ifstream) containing the
//                         glb to be logged.
// @param[in] time - The time to log it at.
// @param[in] channel_name - The channel name to log it on.
// @param[out] logger - The logger to log to.
void visualize_world_glb(
    const std::istream &glb_stream,
    const time::Timestamp &time,
    const std::string &channel_name,
    InOut<LoggerInterface> logger);

}  // namespace resim::visualization::log
