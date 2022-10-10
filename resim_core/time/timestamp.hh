//
// timestamp.hh
//
// Resim's definition/alias for a timestamp which should be easily convertible
// to other common timestamp formats.
//
#pragma once

#include <chrono>

namespace resim::time {

using Duration = std::chrono::nanoseconds;
using Timestamp = std::chrono::sys_time<Duration>;

}  // namespace resim::time
