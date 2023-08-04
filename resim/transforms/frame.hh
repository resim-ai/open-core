// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/utils/uuid.hh"

namespace resim::transforms {

// Frame class. All transforms are transformations between two frames. Often
// this is implicit. This class helps make it explicit. A Frame is a unique
// object - identifiable by its id_ - that can be used as a label for a frame of
// interest. For example.
// Frame<3> global_frame = Frame<3>::new_frame();
// Frame<3> robot_frame = Frame<3>::new_frame();
// Other libraries (e.g. se3.hh) use Frame objects to help explicit checking of
// Frames, for example in verifying the correctness of transform compositions.
template <const unsigned int dims>
class Frame {
 public:
  // The dimensionality of the frames space e.g. 2,3.
  static constexpr unsigned int DIMS = dims;

  Frame() = default;
  // Initialize a frame with an existing UUID.
  explicit Frame(UUID id) : id_(id) {}
  // Generate a new frame with a new UUID.
  static Frame<DIMS> new_frame() { return Frame(UUID::new_uuid()); }
  static Frame<DIMS> null_frame() { return Frame(UUID()); }

  // Is this frame the same as another?
  bool operator==(const Frame<DIMS> &other) const { return id_ == other.id_; }
  bool operator!=(const Frame<DIMS> &other) const { return id_ != other.id_; }
  // Test is this frame is a NULL frame (id_ = {0})
  bool is_null() const { return id_.is_null(); }
  // Retrieve the id of the frame.
  const UUID &id() const { return id_; }

 private:
  UUID id_{};
};

}  // namespace resim::transforms
