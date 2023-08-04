// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#include <string_view>
#pragma once

namespace resim::simulator {

constexpr std::string_view TIME_TOPIC = "time";
constexpr std::string_view ACTOR_STATES_TOPIC = "actor_states";
constexpr std::string_view ACTOR_GEOMETRIES_TOPIC = "actor_geometries";
constexpr std::string_view SCHEDULE_TIMELORD_TOPIC = "schedule";

// A null topic which no-one should depend on. This is used as a provision for
// tasks that no other tasks should depend on.
constexpr std::string_view NULL_TOPIC = "null";

}  // namespace resim::simulator
