
#pragma once

#include <filesystem>

#include "resim/time/timestamp.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/mcap_logger.hh"

namespace resim::visualization::log {

void visualize_world_glb(
    const std::filesystem::path &world_glb_path,
    const time::Timestamp &time,
    InOut<LoggerInterface> logger);

}
