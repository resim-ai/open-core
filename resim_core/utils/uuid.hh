#pragma once

#include <uuid/uuid.h>

#include <array>

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

  // Generate a new UUID.
  static UUID new_uuid();

  // Compate this UUID with another.
  bool operator==(const UUID &other) const;
  bool operator!=(const UUID &other) const;

  // Access the raw underlying id.
  const std::array<unsigned char, ARRAY_SIZE> &id() const;

 private:
  // The id data.
  std::array<unsigned char, ARRAY_SIZE> id_;
};

}  // namespace resim
