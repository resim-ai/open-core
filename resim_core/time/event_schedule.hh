#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

#include "resim_core/assert/assert.hh"
#include "resim_core/time/timestamp.hh"

namespace resim::time {

//
// This class stores a time-ordered queue of events with arbitrary payloads. It
// can be used to efficiently queue future events and pull time-ordere events
// off the queue one at at time. This queue is stable in the sense that it's
// first-in-first-out for events with the same timestamp. This limits this
// object to having no more than std::numeric_limits<Count_t>::max() events
// during its lifetime. We accordingly use uint64_t as our default Count_t since
// this should be more than enough for our applications.
//
template <typename Payload, typename Count_t = uint64_t>
class EventSchedule {
 public:
  struct Event {
    Timestamp time;
    Payload payload;
  };

  // The maximimum number of events that can be scheduled without wrap-around
  // issues when sorting events in the priority queue.
  static constexpr std::size_t MAX_EVENTS = std::numeric_limits<Count_t>::max();

  // Constructor
  EventSchedule();

  // Schedule an event for some time. This class will not check whether this
  // event is in the "future" since it doesn't store the current time
  // internally.
  // @param[in] time - When to schedule the future event.
  // @param[in] payload - A payload for this time.
  void schedule_event(Timestamp time, Payload payload);

  // Access the top event in the queue.
  const Event &top_event() const;

  // Pop the next event off the queue.
  void pop_event();

  // Get the number of scheduled events.
  std::size_t size() const;

 private:
  // Indexed version of events so that we don't have undefined order
  // for equal-time events.
  struct IndexedEvent : public Event {
    IndexedEvent(Timestamp time, Payload payload, Count_t insertion_count);

    Count_t insertion_count = 0U;
  };

  // This functor class compares two events and orders them.
  class CompareEvents {
   public:
    bool operator()(const IndexedEvent &lhs, const IndexedEvent &rhs) const;
  };

  Count_t insertion_count_ = 0U;
  std::priority_queue<IndexedEvent, std::vector<IndexedEvent>, CompareEvents>
      event_queue_;
};

template <typename Payload, typename Count_t>
EventSchedule<Payload, Count_t>::EventSchedule()
    : event_queue_{CompareEvents{}} {}

template <typename Payload, typename Count_t>
void EventSchedule<Payload, Count_t>::schedule_event(
    const Timestamp time,
    Payload payload) {
  constexpr auto ERROR = "Can't schedule in exhausted EventSchedule.";
  REASSERT(insertion_count_ < MAX_EVENTS, ERROR);
  event_queue_.emplace(time, std::move(payload), insertion_count_);
  ++insertion_count_;
}

template <typename Payload, typename Count_t>
const typename EventSchedule<Payload, Count_t>::Event &
EventSchedule<Payload, Count_t>::top_event() const {
  return event_queue_.top();
}

template <typename Payload, typename Count_t>
void EventSchedule<Payload, Count_t>::pop_event() {
  event_queue_.pop();
}

template <typename Payload, typename Count_t>
std::size_t EventSchedule<Payload, Count_t>::size() const {
  return event_queue_.size();
}

template <typename Payload, typename Count_t>
EventSchedule<Payload, Count_t>::IndexedEvent::IndexedEvent(
    const Timestamp time,
    Payload payload,
    const Count_t insertion_count)
    : Event{time, std::move(payload)},
      insertion_count{insertion_count} {}

template <typename Payload, typename Count_t>
bool EventSchedule<Payload, Count_t>::CompareEvents::operator()(
    const IndexedEvent &lhs,
    const IndexedEvent &rhs) const {
  if (lhs.time > rhs.time) {
    return true;
  }
  // Tied time case.
  return lhs.time == rhs.time and lhs.insertion_count > rhs.insertion_count;
}

}  // namespace resim::time
