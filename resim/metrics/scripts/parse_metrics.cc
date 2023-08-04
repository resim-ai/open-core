// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cxxopts.hpp>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>
#include <string>

#include "nlohmann/json.hpp"
#include "resim/assert/assert.hh"
#include "resim/metrics/proto/simple_metric.pb.h"
#include "resim/metrics/proto/simple_metric_to_proto.hh"
#include "resim/metrics/simple_metric.hh"
#include "resim/time/timestamp.hh"

using json = nlohmann::json;

namespace resim::metrics::scripts {
namespace {
const std::string METRIC_PREFIX("metric_");
}  // namespace

// This particular parser assumes the following:
// - All metrics are prefixed with "metric_" in their channel name
// - All such channels are serialized by metrics::proto::SimpleMetric.
// It then writes a json storing
// {
//  "metrics": {
//     channel: [
//       {
//         "metric_value": (double|nullptr),
//         "time_s": double,
//         "reported": (metric_value != nullptr)
//       },
//       ...
//     ]
//   }
// }
//
// TODO(tknowles): Next time we work on this, we should consider
// defining a more concrete schema in protobuf, and then using
// the google::protobuf::util::MessageToJsonString method,
// rather than the full nlohmann/json library. This is an
// intermediate, as we don't want to make schemas concrete yet.
int parse_metrics(
    const std::filesystem::path& mcap_file_path,
    const std::filesystem::path& json_file_path) {
  json metrics{};
  metrics["metrics"] = {};

  mcap::McapReader reader;
  REASSERT(reader.open(mcap_file_path.string()).ok());

  // Read all matching messages
  for (const mcap::MessageView& view : reader.readMessages()) {
    if (view.channel->topic.starts_with(METRIC_PREFIX)) {
      proto::SimpleMetric msg;
      REASSERT(msg.ParseFromArray(
          static_cast<const void*>(view.message.data),
          view.message.dataSize));
      const SimpleMetric m = unpack(msg);
      json metric_entry{};
      metric_entry["time_s"] = time::as_seconds(m.time.time_since_epoch());

      if (m.actor_id.has_value()) {
        metric_entry["actor_id"] = m.actor_id.value().to_string();
      } else {
        metric_entry["actor_id"] = nullptr;
      }

      if (m.metric_value.has_value()) {
        metric_entry["metric_value"] = m.metric_value.value();
        metric_entry["reported"] = true;
      } else {
        metric_entry["metric_value"] = nullptr;
        metric_entry["reported"] = false;
      }
      metrics["metrics"][m.name].push_back(metric_entry);
    }
  }

  std::ofstream o(json_file_path.string());
  o << std::setw(2) << metrics << std::endl;

  std::cout << "Successfully wrote metrics from " << mcap_file_path << " to "
            << json_file_path << std::endl;
  return 0;
}

}  // namespace resim::metrics::scripts

int main(int argc, char* argv[]) {
  try {
    // Parse options
    cxxopts::Options options(
        "parse_metrics",
        "Parses metrics from an .mcap file into a metrics.json file");
    options.positional_help("").show_positional_help();

    options.add_options()(
        "i,input_mcap",
        "Input mcap path",
        cxxopts::value<std::filesystem::path>())(
        "o,output_json",
        "Output json",
        cxxopts::value<std::filesystem::path>())("h,help", "Print usage");
    options.parse_positional({"input_mcap", "output_json"});
    cxxopts::ParseResult result = options.parse(argc, argv);

    // Catch help option
    if (result.count("help") != 0U) {
      std::cout << options.help({""}) << std::endl;
      return 0;
    }

    // Read and check mcap path
    std::filesystem::path input_mcap{};
    if (result.count("input_mcap") != 0U) {
      input_mcap = result["input_mcap"].as<std::filesystem::path>();
      if (input_mcap.extension() != ".mcap") {
        std::cerr << "Error: The input_mcap arg should be a file path ending "
                     "with '.mcap'."
                  << std::endl;
        return 1;
      }
      if (!std::filesystem::exists(input_mcap)) {
        std::cerr << "Error: input mcap does not exist: " << input_mcap
                  << std::endl;
      }
    } else {
      std::cout << "Error: input_mcap is required\n\n"
                << options.help({""}) << std::endl;
      return 1;
    }

    // Read and check mcap path
    std::filesystem::path output_json{};
    if (result.count("output_json") != 0U) {
      output_json = result["output_json"].as<std::filesystem::path>();
      if (output_json.extension() != ".json") {
        std::cerr << "Error: The output_json arg should be a file path ending "
                     "with '.json'."
                  << std::endl;
        return 1;
      }
      if (std::filesystem::exists(output_json)) {
        std::cerr << "Error: output_json already exists: " << output_json
                  << std::endl;
        return 1;
      }
    } else {
      std::cout << "Error: output_json is required\n\n"
                << options.help({""}) << std::endl;
      return 1;
    }

    // Run metrics parsing
    resim::metrics::scripts::parse_metrics(input_mcap, output_json);

    return 0;
  } catch (std::exception) {
    std::cerr << "Error while parsing metrics" << std::endl;
    return 1;
  }
}
