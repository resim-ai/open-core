// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <istream>
#include <mcap/reader.hpp>

namespace resim {

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
}  // namespace resim
