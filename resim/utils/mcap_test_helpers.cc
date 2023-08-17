
#include "resim/utils/mcap_test_helpers.hh"

#include <cstring>
#include <string>

namespace resim {

StreamReader::StreamReader(std::istream& stream) : stream_{stream} {
  stream_.seekg(0, std::istream::end);
  size_ = stream_.tellg();
  stream_.seekg(0, std::istream::beg);
}

uint64_t StreamReader::size() const { return size_; }

uint64_t
StreamReader::read(std::byte** output, uint64_t offset, uint64_t size) {
  if (offset >= size_) {
    return 0;
  }

  if (offset != position_) {
    stream_.seekg(static_cast<std::streamoff>(offset));
    position_ = offset;
  }

  if (size > buffer_.size()) {
    buffer_.resize(size);
  }

  // We need to use memcpy to avoid a pointer cast, which clang-tidy doesn't
  // like:
  std::string tmp(size, '\0');
  stream_.read(tmp.data(), static_cast<std::streamoff>(size));
  std::memcpy(buffer_.data(), tmp.data(), size);
  *output = buffer_.data();

  const uint64_t bytesRead = stream_.gcount();
  position_ += bytesRead;
  return bytesRead;
}
}  // namespace resim
