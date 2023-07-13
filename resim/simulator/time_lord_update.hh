#pragma once

#include <tuple>

#include "resim/time/timestamp.hh"

namespace resim::simulator {

struct TimeLordUpdate {
  // These enum elements must be ordered from first in priority to last in
  // priority
  enum UpdateBehaviour { FULL_UPDATE_THEN_TERMINATE, FULL_UPDATE, UNSUPPORTED };

  // These enum elements must be ordered from first in priority to last in
  // priority
  enum UpdateType { UNIT_SCHEDULED, PRESCHEDULED, UNDEFINED };

  time::Timestamp time;
  UpdateBehaviour update_behaviour;
  UpdateType update_type;

  TimeLordUpdate(
      const time::Timestamp& t,
      const UpdateBehaviour& behaviour,
      const UpdateType& type)
      : time(t),
        update_behaviour(behaviour),
        update_type(type){};

  TimeLordUpdate(const TimeLordUpdate&) = default;
  TimeLordUpdate(TimeLordUpdate&&) = default;
  TimeLordUpdate& operator=(const TimeLordUpdate& other) = default;
  TimeLordUpdate& operator=(TimeLordUpdate&& other) = default;
  ~TimeLordUpdate() = default;

  // The ordering used in these comparison operators is extremely important, as
  // it defines the ordering of updates. Only one update will occur with any
  // given time, so it's absolutely crucial that these are correctly
  // implemented.
  friend bool operator==(const TimeLordUpdate& lhs, const TimeLordUpdate& rhs) {
    return std::tie(lhs.time, lhs.update_behaviour, lhs.update_type) ==
           std::tie(rhs.time, rhs.update_behaviour, rhs.update_type);
  }

  friend bool operator!=(const TimeLordUpdate& lhs, const TimeLordUpdate& rhs) {
    return !(lhs == rhs);
  }

  friend bool operator<(const TimeLordUpdate& lhs, const TimeLordUpdate& rhs) {
    return std::tie(lhs.time, lhs.update_behaviour, lhs.update_type) <
           std::tie(rhs.time, rhs.update_behaviour, rhs.update_type);
  }

  friend bool operator>(const TimeLordUpdate& lhs, const TimeLordUpdate& rhs) {
    return rhs < lhs;
  }

  friend bool operator<=(const TimeLordUpdate& lhs, const TimeLordUpdate& rhs) {
    return !(rhs < lhs);
  }

  friend bool operator>=(const TimeLordUpdate& lhs, const TimeLordUpdate& rhs) {
    return !(lhs < rhs);
  }
};

}  // namespace resim::simulator
