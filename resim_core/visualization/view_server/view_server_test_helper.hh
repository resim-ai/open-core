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
template <typename Viewable>
std::vector<Viewable> generate_payload_type(
    unsigned count = detail::MIN_TEST_ELEMENTS);

// Returns the prefix for the ReViewable type's name in Foxglove
template <typename Viewable>
std::string get_type_prefix();

}  // namespace resim::visualization::view_server
