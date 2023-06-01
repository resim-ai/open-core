

#include "resim_core/assert/assert.hh"

#include <fmt/core.h>

#include <utility>

namespace resim {

AssertException::AssertException(
    std::string_view cond_str,
    std::string_view file,
    int line,
    std::string_view message)
    : what_{fmt::format(
          "<{0}:{1}> - ReAssertion failed: ({2}). Message: {3}",
          file,
          line,
          cond_str,
          message)} {}

const char *AssertException::what() const noexcept { return what_.c_str(); }

}  // namespace resim
