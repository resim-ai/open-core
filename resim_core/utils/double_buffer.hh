#pragma once

#include <array>

namespace resim {

// This class is a simple double buffer so that we can update the state of the
// world atomically. We could eventually implement this using a larger circular
// buffer.
template <typename T>
class DoubleBuffer {
 public:
  // Construct the buffer with default current and next elements.
  DoubleBuffer() = default;

  // Construct the buffer with a given current and next elements.
  DoubleBuffer(const T &current, const T &next) : buffers_{current, next} {}

  // Getter for the current element. Note that we don't allow mutable access so
  // that users don't accidentally modify it. Users should only ever update the
  // next element and then swap.
  const T &current() const { return buffers_.at(current_index_); }

  // Getters for the next element.
  const T &next() const { return buffers_.at(next_index()); }
  T &mutable_next() { return buffers_.at(next_index()); }

  // Swap the current and next elements
  void swap() { current_index_ = next_index(); }

 private:
  // Helper which gets the index of the next element
  std::size_t next_index() const { return (current_index_ + 1U) % 2U; }

  std::size_t current_index_{0U};
  std::array<T, 2> buffers_{};
};

}  // namespace resim
