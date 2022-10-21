#include "resim_core/utils/uuid.hh"

#include <glog/logging.h>

namespace resim {

namespace {

constexpr unsigned int ARRAY_SIZE = UUID::ARRAY_SIZE;

}  // namespace

UUID::UUID() : id_{{0}} {}

UUID::UUID(std::array<unsigned char, ARRAY_SIZE> id) : id_(id) {}

UUID::UUID(const std::string &str) : id_{} {
  constexpr int SUCCESS = 0;
  const int result = uuid_parse(str.c_str(), id_.data());
  CHECK(SUCCESS == result) << "Invalid UUID string!";
}

UUID UUID::new_uuid() {
  std::array<unsigned char, ARRAY_SIZE> id{};
  uuid_generate(id.data());
  return UUID(id);
}

std::string UUID::to_string() const {
  // This includes the 36 bytes specified by the uuid library and one extra byte
  // for null termination
  constexpr std::size_t NUM_BYTES = 37;
  std::array<char, NUM_BYTES> unparsed_char_array{};
  uuid_unparse_lower(id_.data(), unparsed_char_array.data());
  return unparsed_char_array.data();
}

bool UUID::operator==(const UUID &other) const { return id_ == other.id_; }

bool UUID::operator!=(const UUID &other) const { return id_ != other.id_; }

const std::array<unsigned char, ARRAY_SIZE> &UUID::id() const { return id_; }

}  // namespace resim
