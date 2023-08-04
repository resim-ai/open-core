// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/utils/status.hh"

#include <fmt/color.h>
#include <fmt/core.h>
#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include <cstdlib>
#include <string>
#include <string_view>

#include "resim/assert/assert.hh"

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

// A helper function which calls RETURN_IF_NOT_OK() so we can test it.
Status return_if_not_ok_function(const Status &s) {
  RETURN_IF_NOT_OK(s);
  return OKAY_STATUS;
}

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

// Test RETURN_IF_NOT_OK to make sure it correctly returns when we pass a bad
// status.
TEST(StatusValueTest, TestReturnIfNotOK) {
  const Status bad_status = MAKE_STATUS("Test Status!");
  EXPECT_EQ(return_if_not_ok_function(bad_status).what(), bad_status.what());

  const Status good_status{OKAY_STATUS};
  EXPECT_EQ(return_if_not_ok_function(good_status).what(), OKAY_STATUS.what());
}

// Ensure that the RETURN_IF_NOT_OK() macro evaluates its argument exactly
// once.
TEST(StatusValueTest, TestReturnIfNotOKCallsOnce) {
  int call_count = 0;
  enum class Arg {
    GOOD = 0,
    BAD,
  };

  const auto f = [&](const Arg arg) -> Status {
    ++call_count;

    if (arg == Arg::GOOD) {
      return OKAY_STATUS;
    }
    return MAKE_STATUS("Bad Status");
  };

  for (const auto &arg : {Arg::GOOD, Arg::BAD}) {
    // Reset the call count
    call_count = 0;
    // Create and instantly call the lambda
    [&]() -> Status {
      RETURN_IF_NOT_OK(f(arg));
      return OKAY_STATUS;
    }();

    EXPECT_EQ(call_count, 1);
  }
}

}  // namespace resim
