// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/experiences/proto/completion_criteria_to_proto.hh"

#include "resim/assert/assert.hh"
#include "resim/experiences/completion_criteria.hh"
#include "resim/experiences/proto/completion_criteria.pb.h"
#include "resim/experiences/proto/location_condition_to_proto.hh"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/time/timestamp.hh"

namespace resim::experiences::proto {

void pack(
    const experiences::CompletionCriteria &in,
    CompletionCriteria *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  time::proto::pack(in.time_limit, out->mutable_time_limit());
  for (const experiences::Condition &condition : in.conditions) {
    Condition *const proto_condition = out->add_conditions();
    time::proto::pack(condition.delay, proto_condition->mutable_delay());
    pack(condition.condition, proto_condition->mutable_location_condition());
  }
}

experiences::CompletionCriteria unpack(const CompletionCriteria &in) {
  experiences::CompletionCriteria result;
  result.time_limit = time::proto::unpack(in.time_limit());
  for (const Condition &condition_proto : in.conditions()) {
    experiences::Condition condition;
    const time::Duration &delay{time::proto::unpack(condition_proto.delay())};
    condition.delay = delay;
    condition.condition = unpack(condition_proto.location_condition());
    result.conditions.emplace_back(condition);
  }
  return result;
}

}  // namespace resim::experiences::proto
