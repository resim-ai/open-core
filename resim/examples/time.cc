// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// This file is a companion for the Time documentation at
// https://docs.resim.ai/open-core/time/time/

#include <cmath>
#include <cstdlib>
#include <iostream>

#include "resim/assert/assert.hh"
#include "resim/time/event_schedule.hh"
#include "resim/time/sample_interval.hh"
#include "resim/time/timestamp.hh"

using resim::time::Duration;
using resim::time::SecsAndNanos;
using resim::time::Timestamp;

namespace rt = resim::time;

template <typename T>
using EventSchedule = rt::EventSchedule<T>;

int main(int argc, char **argv) {
  //////////////////////////////////////////////////////////////////////////////
  // Basic Types & Converters
  //////////////////////////////////////////////////////////////////////////////
  Duration my_duration{std::chrono::nanoseconds(10000U)};

  double my_duration_s = rt::as_seconds(my_duration);

  // Should pass since my_duration is small
  REASSERT(rt::as_duration(my_duration_s) == my_duration);

  my_duration += std::chrono::system_clock::now().time_since_epoch();

  my_duration_s = rt::as_seconds(my_duration);

  // Will likely not pass because my_duration is big and precision is lost
  // converting it to a double.
  // REASSERT(rt::as_duration(my_duration_s) == my_duration);

  //////////////////////////////////////////////////////////////////////////////
  // Seconds and Nanoseconds
  //////////////////////////////////////////////////////////////////////////////
  const SecsAndNanos my_secs_and_nanos = rt::to_seconds_and_nanos(my_duration);
  REASSERT(rt::from_seconds_and_nanos(my_secs_and_nanos) == my_duration);

  //////////////////////////////////////////////////////////////////////////////
  // Event Scheduling
  //////////////////////////////////////////////////////////////////////////////
  EventSchedule<std::string> my_messages;
  my_messages.schedule_event(Timestamp(std::chrono::nanoseconds(200)), "ReSim");
  my_messages.schedule_event(Timestamp(std::chrono::nanoseconds(100)), "Hello");
  my_messages.schedule_event(Timestamp(std::chrono::nanoseconds(200)), "user!");

  std::cout.precision(9);
  std::cout << std::fixed;
  while (my_messages.size() > 0U) {
    const auto event = my_messages.top_event();
    std::cout << "[" << event.time.time_since_epoch().count() / 1e9 << "] "
              << event.payload << std::endl;
    my_messages.pop_event();
  }

  //////////////////////////////////////////////////////////////////////////////
  // Interval Sampling
  //////////////////////////////////////////////////////////////////////////////
  const Timestamp start_time;
  const Timestamp end_time{start_time + std::chrono::seconds(30)};
  const Duration max_dt = std::chrono::microseconds(100);

  // A stand in function for illustration purposes.
  const auto deviation = [](const Timestamp &t) {
    return std::cos(std::sqrt(rt::as_seconds(t.time_since_epoch())));
  };

  double integral = 0.;
  rt::sample_interval(start_time, end_time, max_dt, [&](const Timestamp &t) {
    // Left rectangles so we leave off the value at the end time
    if (t != end_time) {
      integral += deviation(t);
    }
  });

  // The actual dt per rectangle is the total interval divided by (N - 1)
  const int N = rt::num_samples(start_time, end_time, max_dt);
  const double dt_s = rt::as_seconds(end_time - start_time) / (N - 1);
  integral *= dt_s;

  // Make sure we match the analytical result
  const double analytical_result =
      2. * (-1. + std::cos(std::sqrt(30.)) +
            std::sqrt(30.) * std::sin(std::sqrt(30.)));
  REASSERT(std::fabs(integral - analytical_result) < 1e-4);

  return EXIT_SUCCESS;
}
