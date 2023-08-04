// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <exception>
#include <string>
#include <string_view>

namespace resim {

// An exception type to throw when an assertion fails to be
// true. Users should not throw this directly, but should use the
// REASSERT() convenience macro below.
class AssertException : public std::exception {
 public:
  // Constructor
  // @param[in] cond_str - A string representation of the condition that failed.
  // @param[in] file - The file where the assertion failed.
  // @param[in] line - The line where the assertion failed.
  // @param[in] message - A message to go along with the failed assertion.
  AssertException(
      std::string_view cond_str,
      std::string_view file,
      int line,
      std::string_view message);

  AssertException() = delete;
  AssertException(const AssertException &) = default;
  AssertException(AssertException &&) = default;
  AssertException &operator=(const AssertException &) = default;
  AssertException &operator=(AssertException &&) = default;
  ~AssertException() override = default;

  const char *what() const noexcept override;

 private:
  std::string what_;
};

namespace detail {

// Implementation for assertions. Just throws if the condition is not true.
constexpr void assert_impl(
    bool cond,
    std::string_view cond_str,
    std::string_view file,
    int line,
    std::string_view message = "") {
  if (not cond) {
    throw AssertException(cond_str, file, line, message);
  }
}

}  // namespace detail
}  // namespace resim

// NOLINTBEGIN(cppcoreguidelines-macro-usage)

// Helper macro to help us select the right "overload" of the macro for cases
// when the user decides not to pass a message to the assertion.
#define SELECT_ASSERTER(_1, _2, ASSERTER, ...) ASSERTER

// "Overload" for the case where no message is provided.
#define REASSERT_1(cond) \
  resim::detail::assert_impl(cond, #cond, __FILE__, __LINE__)

// "Overload" for the case where a message is provided.
#define REASSERT_2(cond, message) \
  resim::detail::assert_impl(cond, #cond, __FILE__, __LINE__, message)

// The main assertion macro that users should use.
#define REASSERT(...) \
  SELECT_ASSERTER(__VA_ARGS__, REASSERT_2, REASSERT_1)(__VA_ARGS__)
// NOLINTEND(cppcoreguidelines-macro-usage)
