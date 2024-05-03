
#pragma once

#include "resim/visualization/log/visualize_world_glb.hh"

namespace resim::visualization::log {

void visualize_world_glb(
    const std::filesystem::path &world_glb_path,
    const time::Timestamp &time,
    InOut<LoggerInterface> logger) {
  // TODO(michael)
}

}  // namespace resim::visualization::log
