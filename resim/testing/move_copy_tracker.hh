// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

namespace resim::testing {

// This is a very simple class that can be used to track how many copies or
// moves it undergoes since its inception. This can be useful for testing
// purposes.

class MoveCopyTracker {
 public:
  // Constructors
  MoveCopyTracker() = default;
  ~MoveCopyTracker() = default;
  MoveCopyTracker(const MoveCopyTracker &other);
  MoveCopyTracker(MoveCopyTracker &&other) noexcept;

  // Assignment Operators
  MoveCopyTracker &operator=(const MoveCopyTracker &other);
  MoveCopyTracker &operator=(MoveCopyTracker &&other) noexcept;

  // Getters
  int num_moves() const;
  int num_copies() const;

 private:
  int num_moves_ = 0;
  int num_copies_ = 0;
};

}  // namespace resim::testing
