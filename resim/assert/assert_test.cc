#include "resim/assert/assert.hh"

#include <gtest/gtest.h>

namespace resim {

TEST(CheckTest, TestCheck) {
  EXPECT_THROW(REASSERT(false), AssertException);
  EXPECT_THROW(REASSERT(false, "My message!"), AssertException);
  EXPECT_NO_THROW(REASSERT(true));
}

TEST(CheckTest, TestAssertException) {
  constexpr int SOME_LINE = 42;
  const AssertException check_exception{
      "false condition",
      "some_file.cc",
      SOME_LINE,
      "The flux capacitor broke!"};

  EXPECT_EQ(
      std::string(check_exception.what()),
      "<some_file.cc:42> - ReAssertion failed: (false condition). Message: The "
      "flux capacitor broke!");
}

}  // namespace resim
