#pragma once

#include <string>
#include <vector>

namespace resim::visualization::view_server {

namespace detail {
constexpr unsigned MIN_TEST_ELEMENTS = 7;
}

// Builds and returns a vector containing a fixed number of ReViewable types
// @param[in] count - Optionally, the number of vectors to return. The default
//                    and the minimum are both seven. If you request less the
//                    function will check-fail.
// @param[in] framed - For types where it's relevant, enforce that the output
//                     has associated coordinate frames.
template <typename Viewable>
std::vector<Viewable> generate_payload_type(
    unsigned count = detail::MIN_TEST_ELEMENTS,
    bool framed = true);

}  // namespace resim::visualization::view_server
