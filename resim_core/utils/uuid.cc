#include "resim_core/utils/uuid.hh"

namespace resim {

namespace {

constexpr unsigned int ARRAY_SIZE = UUID::ARRAY_SIZE;

}  // namespace

UUID::UUID() : id_{{0}} {}

UUID::UUID(std::array<unsigned char, ARRAY_SIZE> id) : id_(id) {}

UUID UUID::new_uuid() {
  std::array<unsigned char, ARRAY_SIZE> id{};
  uuid_generate(id.data());
  return UUID(id);
}

bool UUID::operator==(const UUID &other) const { return id_ == other.id_; }

bool UUID::operator!=(const UUID &other) const { return id_ != other.id_; }

const std::array<unsigned char, ARRAY_SIZE> &UUID::id() const { return id_; }

}  // namespace resim
