
#pragma once

#include <mcap/reader.hpp>
#include <string>

#include "resim/experiences/experience.hh"
#include "resim/experiences/proto/experience.pb.h"

namespace resim::visualization::log {

// Helper to generate a test experience config containing a valid header and a
// single actor with a Wireframe geometry.
experiences::Experience test_experience();

// Helper to make an mcap log (written to a string) with num_messages copies of
// experience_msg logged to the "/experience" channel.
// @param[in] experience_msg - The message to write to the "/experience"
//                             channel.
// @param[in] num_messages - The number of times to write experience_msg to the
//                           "/experience" channel.
std::string make_input_log(
    const experiences::proto::Experience& experience_msg,
    int num_messages = 1);

// A more generic version of mcap::FileStreamReader so we can read a log from a
// string. This implements the basic functionality required by mcap::IReadable
// using the stored std::istream.
class StreamReader final : public mcap::IReadable {
 public:
  explicit StreamReader(std::istream& stream);

  uint64_t size() const override;
  uint64_t read(std::byte** output, uint64_t offset, uint64_t size) override;

 private:
  std::istream& stream_;
  std::vector<std::byte> buffer_;
  uint64_t size_{};
  uint64_t position_{};
};

}  // namespace resim::visualization::log
