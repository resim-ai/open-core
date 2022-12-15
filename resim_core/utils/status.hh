
#pragma once

#include <fmt/color.h>
#include <fmt/core.h>
#include <glog/logging.h>

#include <string>
#include <string_view>
#include <variant>

namespace resim {
//
// This simple class represents a status that can be returned from a subroutine
// to indicate the success or failure of that subroutine. This class is designed
// such that it does not perform heap allocations unless the user calls the
// "what()" member function which is used to retrieve a formatted string
// representing the error.
//
class Status {
 public:
  Status() = default;
  ~Status() = default;
  Status(const Status &) = default;
  Status(Status &&) = default;
  Status &operator=(const Status &) = default;
  Status &operator=(Status &&) = default;

  // Make a bad status formatted with the given file name and line number. Users
  // should not use this member function directly and should instead use the
  // MAKE_STATUS() macro below.
  // @param[in] file - The file where this bad status was created.
  // @param[in] line - The line where this bad status was created.
  // @param[in] message - The message associated with this bad status.
  static constexpr Status
  make_at_line(const char *file, int line, const char *message);

  // Get whether this Status is good or bad.
  constexpr bool ok() const;

  // Produce a formatted string representing this Status. This may
  // cause a heap allocation.
  std::string what() const;

 private:
  struct OkayType {
    static constexpr std::string_view MESSAGE = "OKAY";
  };

  struct ErrType {
    std::string_view message;

    // Information about where the error is located
    std::string_view file;
    int line = 0;
  };

  using StatusVariant = std::variant<OkayType, ErrType>;

  explicit constexpr Status(StatusVariant status);

  // Default is okay
  StatusVariant status_;
};

// Static constant representing an OK status.
constexpr Status OKAY_STATUS{};

// Define a macro to make new statuses that carry file and line-number
// information about where they're created.
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MAKE_STATUS(msg) (Status::make_at_line(__FILE__, __LINE__, msg))

// Define a macro to check that a given status is okay.
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define CHECK_STATUS_OK(status)       \
  do {                                \
    const auto err_msg = fmt::format( \
        fg(fmt::color::red),          \
        "{{{0}.what() == {1}}}",      \
        #status,                      \
        (status).what());             \
    CHECK((status).ok()) << err_msg;  \
  } while (0)

constexpr Status
Status::make_at_line(const char *file, int line, const char *message) {
  return Status{StatusVariant{ErrType{
      .message = message,
      .file = file,
      .line = line,
  }}};
}

constexpr Status::Status(StatusVariant status) : status_{status} {}

constexpr bool Status::ok() const {
  return std::holds_alternative<OkayType>(status_);
}

}  // namespace resim
