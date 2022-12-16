//
// timestamp.hh
//
// Resim's definition/alias for a timestamp which should be easily convertible
// to other common timestamp formats.
//
#pragma once

#include <chrono>
#include <cstdint>
#include <ratio>

namespace resim::time {

using Duration = std::chrono::nanoseconds;
using Timestamp = std::chrono::sys_time<Duration>;

// Convert a duration to seconds as a double
constexpr double as_seconds(const Duration duration) {
  return std::chrono::duration<double>(duration).count();
}

// Create a duration from seconds as a double
constexpr Duration as_duration(const double t_s) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(t_s));
}

// A timestamp represented as integer seconds and nanoseconds. This
// representation is common in robotics frameworks (ROS) and elsewhere
// (e.g.
// https://developers.google.com/protocol-buffers/docs/reference/google.protobuf#timestamp)
struct SecsAndNanos {
  int64_t secs = 0;
  int32_t nanos = 0;
};

// Convert a duration to SecsAndNanos
constexpr SecsAndNanos to_seconds_and_nanos(const Duration duration) {
  return SecsAndNanos{
      .secs = duration.count() / std::nano::den,
      .nanos = static_cast<int32_t>(duration.count() % std::nano::den),
  };
}

// Convert SecsAndNanos to a duration
constexpr Duration from_seconds_and_nanos(const SecsAndNanos &secs_and_nanos) {
  return Duration{secs_and_nanos.secs * std::nano::den + secs_and_nanos.nanos};
}

}  // namespace resim::time
