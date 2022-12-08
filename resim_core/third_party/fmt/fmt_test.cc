#include <fmt/core.h>
#include <gtest/gtest.h>

namespace resim {

TEST(FmtTest, TestFormat) {
  // SETUP / ACTION / VERIFICATION
  EXPECT_EQ(fmt::format("Hello, {}!", "World"), "Hello, World!");
}

}  // namespace resim
