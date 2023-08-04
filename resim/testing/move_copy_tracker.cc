// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/testing/move_copy_tracker.hh"

namespace resim ::testing {

MoveCopyTracker::MoveCopyTracker(const MoveCopyTracker &other)
    : num_moves_{other.num_moves_},
      num_copies_{other.num_copies_ + 1} {}

MoveCopyTracker::MoveCopyTracker(MoveCopyTracker &&other) noexcept
    : num_moves_{other.num_moves_ + 1},
      num_copies_{other.num_copies_} {}

MoveCopyTracker &MoveCopyTracker::operator=(const MoveCopyTracker &other) {
  num_moves_ = other.num_moves_;
  num_copies_ = other.num_copies_ + 1;
  return (*this);
}
MoveCopyTracker &MoveCopyTracker::operator=(MoveCopyTracker &&other) noexcept {
  num_moves_ = other.num_moves_ + 1;
  num_copies_ = other.num_copies_;
  return (*this);
}

int MoveCopyTracker::num_moves() const { return num_moves_; }
int MoveCopyTracker::num_copies() const { return num_copies_; }

}  // namespace resim::testing
