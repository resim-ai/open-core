#pragma once

#include <uuid/uuid.h>

#include <array>
#include <string>

namespace resim {

// Simple UUID wrapper class that uses Linux lib uuid.h under the hood, but has
// an API that is more 'modern' c++ complaint and does not rely so much on raw
// pointers and implicit pointer decay on the array.
class UUID {
 public:
  static constexpr unsigned int ARRAY_SIZE = 16;
  // Intitialize with empty id.
  UUID();

  // Construct from a raw id.
  explicit UUID(std::array<unsigned char, ARRAY_SIZE> id);

  // Construct from an ASCII string representation
  explicit UUID(const std::string &str);

  // Generate a new UUID.
  static UUID new_uuid();

  // Format this UUID as a string
  std::string to_string() const;

  // Compate this UUID with another.
  bool operator==(const UUID &other) const;
  bool operator!=(const UUID &other) const;

  // Test whether this uuid is null (i.e {0})
  bool is_null() const;

  // Access the raw underlying id.
  const std::array<unsigned char, ARRAY_SIZE> &id() const;

 private:
  // The id data.
  std::array<unsigned char, ARRAY_SIZE> id_;
};

}  // namespace resim

namespace std {

template <>
struct hash<resim::UUID> {
  std::size_t operator()(const resim::UUID &k) const {
    return std::hash<std::string>()(k.to_string());
  }
};

}  // namespace std
