#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include <cstdlib>
#include <cxxopts.hpp>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "resim_core/assert/assert.hh"
#include "resim_core/experiences/experience.hh"
#include "resim_core/experiences/proto/experience.pb.h"
#include "resim_core/experiences/proto/experience_to_proto.hh"
#include "resim_core/simulator/simulate.hh"

namespace resim::simulator {

namespace {

// A small helper to read and unpack the config
// @param[in] exp_path - The path to read the experience from.
experiences::Experience load_experience(const std::filesystem::path &exp_path) {
  REASSERT(std::filesystem::exists(exp_path), "Experience config not found!");

  experiences::proto::Experience experience_msg;
  {
    std::ifstream exp_stream(exp_path);
    std::stringstream buffer;
    buffer << exp_stream.rdbuf();
    const bool success = google::protobuf::TextFormat::ParseFromString(
        buffer.str(),
        &experience_msg);
    REASSERT(success, "Failed to open experience config!");
  }
  return unpack(experience_msg);
}
}  // namespace

void run_sim(int argc, char **argv) {
  cxxopts::Options options{"ReSim Run", "A simple program for running sims."};
  // clang-format off
  options.add_options()
    ("l,log", "Log location (required)", cxxopts::value<std::string>())
    ("c,config", "Config location (required)", cxxopts::value<std::string>())
    ("h,help", "Print usage")
  ;
  // clang-format on
  auto options_result = options.parse(argc, argv);

  if (options_result.count("help") > 0U or options_result.count("log") == 0U or
      options_result.count("config") == 0U) {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  const std::filesystem::path mcap_path{
      options_result["log"].as<std::string>()};
  const std::filesystem::path experience_path{
      options_result["config"].as<std::string>()};

  const auto experience = load_experience(experience_path);
  simulate(experience, mcap_path);
}

}  // namespace resim::simulator

int main(int argc, char **argv) {
  try {
    resim::simulator::run_sim(argc, argv);
  } catch (const std::exception &e) {
    std::cerr << "Exception thrown during sim run!" << std::endl;
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
