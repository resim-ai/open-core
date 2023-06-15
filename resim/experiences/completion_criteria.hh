#pragma once

#include <vector>

#include "resim/experiences/location_condition.hh"
#include "resim/time/timestamp.hh"

namespace resim::experiences {

// Right now we only have one type of condition, which can be used alongside a
// delay to capture additional simulation.
struct Condition {
  LocationCondition condition;
  time::Duration delay{};
};

// A completion criteria is a set of conditions that would trigger the
// experience to end. Each experience has a time limit, but an optional list of
// conditions may also terminate the experience.
struct CompletionCriteria {
  time::Duration time_limit{};
  std::vector<Condition> conditions;
};

}  // namespace resim::experiences
