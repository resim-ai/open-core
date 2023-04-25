
#include <string_view>
#pragma once

namespace resim::simulator {

constexpr std::string_view TIME_TOPIC = "time";
constexpr std::string_view ACTOR_STATES_TOPIC = "actor_states";

// A null topic which no-one should depend on. This is used as a provision for
// tasks that no other tasks should depend on.
constexpr std::string_view NULL_TOPIC = "null";

}  // namespace resim::simulator
