
#include <fmt/core.h>

#include <cstdint>
#include <cstdlib>
#include <cxxopts.hpp>
#include <limits>
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>

#include "resim/assert/assert.hh"
#include "resim/utils/snippet_mcap.hh"

namespace resim {

int trim_mcap(int argc, char **argv) {
  cxxopts::Options options{
      "trim_mcap",
      "A simple program for trimming mcap logs.\n\nSaves a log to <output> "
      "containing all messages from <input> (along with channels and schemas) "
      "which fall within the interval [start_time, end_time]. Does not yet "
      "support copying over attachments."};

  const auto default_start = std::to_string(0.0);
  const auto default_end = fmt::format(
      "{:.9f}",
      static_cast<long double>(std::numeric_limits<int64_t>::max()) / 1e9L);

  // clang-format off
  options.add_options()
    ("input", "Input log location", cxxopts::value<std::string>())
    ("output", "Output log location", cxxopts::value<std::string>())
    // cxxopts doesn't natively support floating point
    ("start_time", "Snippet start time", 
      cxxopts::value<std::string>()->default_value(default_start))
    ("end_time", "Snippet end time", 
      cxxopts::value<std::string>()->default_value(default_end))
    ("h,help", "Print usage")
  ;

options.positional_help("<input> <output>").show_positional_help();
  // clang-format on
  options.parse_positional({"input", "output"});
  auto options_result = options.parse(argc, argv);

  if (options_result.count("help") > 0U or
      options_result.count("input") == 0U or
      options_result.count("output") == 0U) {
    std::cout << options.help() << std::endl;
    return EXIT_FAILURE;
  }

  time::Timestamp start_time{time::as_duration(
      std::stold(options_result["start_time"].as<std::string>()))};
  time::Timestamp end_time{time::as_duration(
      std::stold(options_result["end_time"].as<std::string>()))};

  mcap::McapReader input_mcap;
  REASSERT(
      input_mcap.open(options_result["input"].as<std::string>()).ok(),
      "Failed to open input log!");
  REASSERT(
      input_mcap.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  mcap::McapWriter output_mcap;
  const mcap::McapWriterOptions writer_options{
      input_mcap.header().has_value() ? input_mcap.header()->profile
                                      : "resim_mcap"};
  REASSERT(output_mcap
               .open(options_result["output"].as<std::string>(), writer_options)
               .ok());

  snippet_mcap(start_time, end_time, InOut{input_mcap}, InOut{output_mcap});

  return EXIT_SUCCESS;
}

}  // namespace resim

int main(int argc, char **argv) { return resim::trim_mcap(argc, argv); }
