
#include "resim_core/utils/status.hh"

#include <fmt/color.h>
#include <fmt/core.h>
#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include <cstdlib>
#include <string>
#include <string_view>

#include "resim_core/assert/assert.hh"

namespace resim {

namespace {

// An enum to pass to our test function such that the function returns a bad
// status when the bad argument is passed and returns a good status when the
// good argument is passed.
enum class Argument {
  GOOD = 0,
  BAD,
};

// Message to return in the bad status
constexpr auto BAD_STATUS_MESSAGE{"Bad argument passed!"};

// A test function which can fail depending on its argument
Status test_function(const Argument arg) {
  if (arg == Argument::GOOD) {
    return OKAY_STATUS;
  }
  return MAKE_STATUS(BAD_STATUS_MESSAGE);
}
// The following line is exactly four lines past the status definition. Tests
// will fail if this becomes false.
const std::string bad_status_line{std::to_string(__LINE__ - 4)};
const std::string expected_bad_status_what{fmt::format(
    "<{0}:{1}> {2}",
    __FILE__,
    bad_status_line,
    BAD_STATUS_MESSAGE)};

}  // namespace

TEST(StatusTest, TestOk) {
  // SETUP / ACTION / VERIFICATION
  EXPECT_TRUE(test_function(resim::Argument::GOOD).ok());
  EXPECT_TRUE(not test_function(resim::Argument::BAD).ok());
}

TEST(StatusTest, TestWhat) {
  // Okay case:
  const Status good_status{test_function(resim::Argument::GOOD)};
  ASSERT_TRUE(good_status.ok());
  EXPECT_EQ(good_status.what(), "OKAY");

  // Non-okay case:
  const Status bad_status{test_function(resim::Argument::BAD)};
  ASSERT_TRUE(not bad_status.ok());
  EXPECT_EQ(bad_status.what(), expected_bad_status_what);
}

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(StatusDeathTest, TestCheckStatus) {
  // Okay case:
  const Status ok_status{test_function(resim::Argument::GOOD)};
  CHECK_STATUS_OK(ok_status);

  // Non-okay case:
  const Status bad_status{test_function(resim::Argument::BAD)};
  const std::string expected_death_output{fmt::format(
      fg(fmt::color::red),
      "{{bad_status.what() == {}}}",
      expected_bad_status_what)};

  // These macros behave a bit funky if we directly nest them.
  const auto make_check = [&]() { CHECK_STATUS_OK(bad_status); };
  EXPECT_THROW(make_check(), AssertException);
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim
