// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <fmt/core.h>
#include <gtest/gtest.h>

namespace resim {

TEST(FmtTest, TestFormat) {
  // SETUP / ACTION / VERIFICATION
  EXPECT_EQ(fmt::format("Hello, {}!", "World"), "Hello, World!");
}

}  // namespace resim
