// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cstdlib>
#include <cxxopts.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mcap/reader.hpp>
#include <optional>

#include "resim/assert/assert.hh"
#include "resim/experiences/experience.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/mcap_logger.hh"
#include "resim/visualization/log/extract_experience.hh"
#include "resim/visualization/log/visualize_actor_states.hh"
#include "resim/visualization/log/visualize_world_glb.hh"

namespace resim::visualization::log {

// This helper function actually generates the visualization log based on the
// given sim log path.
// @param[in] sim_log - The path of the sim log to read the sim information
//                      from.
// @param[in] vis_log - The path of the visualization log to write to.
// @throws AssertException if the sim log does not exist or can't be opened by
// the mcap reader.
void generate_visualization_log(
    const std::filesystem::path &sim_log,
    const std::filesystem::path &vis_log,
    const std::optional<std::filesystem::path> &maybe_world_glb) {
  REASSERT(std::filesystem::exists(sim_log), "Input log does not exist!");

  mcap::McapReader reader;
  REASSERT(reader.open(sim_log.string()).ok());

  McapLogger logger{vis_log};
  logger.add_log_contents(InOut{reader});

  const experiences::Experience experience{extract_experience(InOut{reader})};
  visualize_actor_states(experience, InOut{reader}, InOut{logger});

  if (maybe_world_glb) {
    auto message_view = reader.readMessages();
    REASSERT(message_view.begin() != message_view.end());
    const auto time =
        time::Timestamp{time::Duration{message_view.begin()->message.logTime}};

    std::ifstream glb_stream;
    REASSERT(std::filesystem::exists(*maybe_world_glb));
    glb_stream.open(*maybe_world_glb, std::ios::binary);
    visualize_world_glb(
        glb_stream,
        time,
        "/world_geometry",
        InOut{static_cast<LoggerInterface &>(logger)});
  }
}

void make_visualization_log(int argc, char **argv) {
  cxxopts::Options options{
      "Make visualization log",
      "A simple program for making a ReSim log visualizable in Foxglove. \n"
      "Existing topics are copied over from the input log and FrameTransforms "
      "\n"
      "and SceneUpdates are added to make things visible in Foxglove."};
  // clang-format off
  options.add_options()
    ("l,log", "Input log location (required)", cxxopts::value<std::string>())
    ("o,output", "Output log location (required)", cxxopts::value<std::string>())
    ("world_glb", "GLB to use for scene geometry (optional)", cxxopts::value<std::string>())    
    ("h,help", "Print usage")
  ;
  // clang-format on
  auto options_result = options.parse(argc, argv);

  if (options_result.count("help") > 0U or options_result.count("log") == 0U or
      options_result.count("output") == 0U) {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  const std::filesystem::path sim_mcap_path{
      options_result["log"].as<std::string>()};

  const std::filesystem::path vis_mcap_path{
      options_result["output"].as<std::string>()};

  std::optional<std::filesystem::path> maybe_world_glb_path;
  if (options_result.count("world_glb") != 0U) {
    maybe_world_glb_path = options_result["world_glb"].as<std::string>();
  }
  generate_visualization_log(
      sim_mcap_path,
      vis_mcap_path,
      maybe_world_glb_path);
}

}  // namespace resim::visualization::log

int main(int argc, char **argv) {
  resim::visualization::log::make_visualization_log(argc, argv);
  return EXIT_SUCCESS;
}
