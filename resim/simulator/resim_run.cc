// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include <chrono>
#include <cstdlib>
#include <cxxopts.hpp>
#include <exception>
#include <filesystem>
#include <fstream>
#include <indicators/cursor_control.hpp>
#include <indicators/progress_spinner.hpp>
#include <iostream>
#include <thread>
#include <unordered_set>

#include "resim/assert/assert.hh"
#include "resim/experiences/actor.hh"
#include "resim/experiences/experience.hh"
#include "resim/experiences/ilqr_drone.hh"
#include "resim/experiences/proto/experience.pb.h"
#include "resim/experiences/proto/experience_to_proto.hh"
#include "resim/simulator/simulate.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/uuid.hh"

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

// This class encapsulates a simple spinner that starts when it is constructed
// and stops when it is destructed.
class SimpleSpinner {
 public:
  // Constructor
  // @param[in] progress_text - The text to display when running.
  // @param[in] complete_text - The text to display when complete.
  SimpleSpinner(std::string_view progress_text, std::string_view complete_text);

  SimpleSpinner(const SimpleSpinner &) = delete;
  SimpleSpinner(SimpleSpinner &&) = delete;
  SimpleSpinner &operator=(const SimpleSpinner &) = delete;
  SimpleSpinner &operator=(SimpleSpinner &&) = delete;

  ~SimpleSpinner();

 private:
  std::vector<std::string> spinner_states_ = {"/", "-", "\\", "|"};
  indicators::ProgressSpinner spinner_;
  std::string_view complete_text_;
  std::thread thread_;
  bool done_ = false;
};

SimpleSpinner::SimpleSpinner(
    std::string_view progress_text,
    std::string_view complete_text)
    : spinner_{indicators::option::PostfixText{progress_text.data()}, indicators::option::ForegroundColor{indicators::Color::yellow}, indicators::option::ShowPercentage{false}, indicators::option::ShowElapsedTime{true}, indicators::option::SpinnerStates{spinner_states_}, indicators::option::FontStyles{std::vector<indicators::FontStyle>{indicators::FontStyle::bold}}},
      complete_text_{complete_text},
      thread_{[this]() {
        while (not done_) {
          if (spinner_.current() == 4) {
            spinner_.set_progress(0);
          } else {
            spinner_.tick();
          }
          constexpr int MS_PER_STATE = 40;
          std::this_thread::sleep_for(std::chrono::milliseconds(MS_PER_STATE));
        }
      }} {
  indicators::show_console_cursor(false);
}

SimpleSpinner::~SimpleSpinner() {
  done_ = true;
  spinner_.set_option(
      indicators::option::ForegroundColor{indicators::Color::green});
  spinner_.set_option(indicators::option::PrefixText{"[DONE]"});
  spinner_.set_option(indicators::option::ShowSpinner{false});
  spinner_.set_option(indicators::option::ShowPercentage{false});
  spinner_.set_option(indicators::option::PostfixText{complete_text_.data()});
  spinner_.mark_as_completed();
  thread_.join();
  indicators::show_console_cursor(true);
}

// Apply the overrided velocity cost to all actors that are SYSTEM_UNDER_TEST
void apply_velocity_cost_override(
    const double cost,
    InOut<experiences::Experience> experience) {
  std::unordered_set<UUID> relevant_actor_ids;
  for (const auto &actor : experience->dynamic_behavior.actors) {
    if (actor.actor_type == experiences::ActorType::SYSTEM_UNDER_TEST) {
      relevant_actor_ids.emplace(actor.id);
    }
  }

  for (auto &movement_model :
       experience->dynamic_behavior.storyboard.movement_models) {
    if (relevant_actor_ids.contains(movement_model.actor_reference)) {
      auto *const maybe_drone_model =
          std::get_if<experiences::ILQRDrone>(&movement_model.model);
      if (maybe_drone_model != nullptr) {
        auto &drone_model = *maybe_drone_model;
        drone_model.velocity_cost = cost;
      }
    }
  }
}

}  // namespace

void run_sim(int argc, char **argv) {
  cxxopts::Options options{"ReSim Run", "A simple program for running sims."};
  // clang-format off
  options.add_options()
    ("l,log", "Log location (required)", cxxopts::value<std::string>())
    ("c,config", "Config location (required)", cxxopts::value<std::string>())
    ("velocity_cost_override",
         "Override for the velocity cost coefficient for the ego. (optional)",
         cxxopts::value<double>())    
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

  auto experience = [&]() {
    SimpleSpinner spinner{"Loading Experience...", "Experience Loaded!"};
    return load_experience(experience_path);
  }();
  if (options_result.count("velocity_cost_override") > 0) {
    apply_velocity_cost_override(
        options_result["velocity_cost_override"].as<double>(),
        InOut{experience});
  }
  {
    SimpleSpinner spinner{"Running Simulation...", "Simulation Complete!"};
    simulate(experience, mcap_path);
  }
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
