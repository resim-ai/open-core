// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/time/event_schedule.hh"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <random>
#include <vector>

#include "resim/assert/assert.hh"

namespace resim::time {

namespace {
using std::literals::chrono_literals::operator""s;

enum class EventTypes {
  EVENT_A = 0,
  EVENT_B,
  EVENT_C,
  EVENT_D,
  EVENT_E,
};
using Schedule = EventSchedule<EventTypes, uint8_t>;
using Event = typename Schedule::Event;

constexpr Timestamp START_TIME{0s};
}  // namespace

// Test that we can push and pop events correctly.
TEST(EventScheduleTest, TestEvents) {
  // SETUP
  const std::vector<Event> events_to_schedule{
      {START_TIME + 1s, EventTypes::EVENT_A},
      {START_TIME + 2s, EventTypes::EVENT_B},
      {START_TIME + 3s, EventTypes::EVENT_C},
      {START_TIME + 4s, EventTypes::EVENT_D},
      {START_TIME + 5s, EventTypes::EVENT_E},
  };

  std::vector<Event> scrambled_events{events_to_schedule};

  // Check that the events come out in order no matter how we scramble them up.
  do {
    Schedule schedule;

    // ACTION
    for (const auto &[time, event_type] : scrambled_events) {
      schedule.schedule_event(time, event_type);
    }

    // VERIFICATION
    for (const auto &[expected_time, expected_event_type] :
         events_to_schedule) {
      const Event &top_event = schedule.top_event();
      EXPECT_EQ(top_event.time, expected_time);
      EXPECT_EQ(top_event.payload, expected_event_type);
      schedule.pop_event();
    }
  } while (std::next_permutation(
      scrambled_events.begin(),
      scrambled_events.end(),
      [&](const auto &a, const auto &b) { return a.time < b.time; }));
}

// Test that the ordering stays the same when times are equal.
TEST(EventScheduleTest, TestOrderingSameTime) {
  const std::vector<Event> events_to_schedule{
      {START_TIME, EventTypes::EVENT_A},
      {START_TIME, EventTypes::EVENT_B},
      {START_TIME, EventTypes::EVENT_C},
      {START_TIME, EventTypes::EVENT_D},
      {START_TIME, EventTypes::EVENT_E},
  };

  std::vector<std::size_t> scrambled_indices{0U, 1U, 2U, 3U, 4U};

  // Check that the events come out in the order they were inserted.
  do {
    Schedule schedule;
    // ACTION
    for (const std::size_t index : scrambled_indices) {
      const auto &[time, event_type] = events_to_schedule.at(index);
      schedule.schedule_event(time, event_type);
    }

    // VERIFICATION
    for (const std::size_t index : scrambled_indices) {
      const auto &[expected_time, expected_event_type] =
          events_to_schedule.at(index);
      const Event &top_event = schedule.top_event();
      EXPECT_EQ(top_event.time, expected_time);
      EXPECT_EQ(top_event.payload, expected_event_type);
      schedule.pop_event();
    }
  } while (std::next_permutation(
      scrambled_indices.begin(),
      scrambled_indices.end()));
}

// Test that we cannot push too many events for the internal counter into the
// schedule
// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(EventScheduleTest, TestDeathOnTooManyEvents) {
  Schedule schedule;

  // Push some random times
  std::uniform_int_distribution<int> dist{-3, 3};
  constexpr std::size_t SEED = 9381U;
  std::mt19937 rng{SEED};
  for (std::size_t ii = 0; ii < Schedule::MAX_EVENTS; ++ii) {
    schedule.schedule_event(
        START_TIME + std::chrono::seconds(dist(rng)),
        EventTypes::EVENT_A);
    schedule.pop_event();
  }
  EXPECT_THROW(
      schedule.schedule_event(START_TIME, EventTypes::EVENT_A),
      AssertException);
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim::time
