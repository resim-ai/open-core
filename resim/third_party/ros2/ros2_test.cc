// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <rcpputils/asserts.hpp>

TEST(Ros2Test, TestLinksRos2) {
  // Check that we can use ros2 code
  rcpputils::assert_true(true);
}
