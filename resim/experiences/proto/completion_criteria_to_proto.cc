#include "resim/experiences/proto/completion_criteria_to_proto.hh"

#include "resim/assert/assert.hh"
#include "resim/experiences/completion_criteria.hh"
#include "resim/experiences/proto/completion_criteria.pb.h"
#include "resim/experiences/proto/location_condition_to_proto.hh"
#include "resim/time/timestamp.hh"

namespace resim::experiences::proto {

void pack(
    const experiences::CompletionCriteria &in,
    CompletionCriteria *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  const time::SecsAndNanos time_limit_secs_and_nanos{
      time::to_seconds_and_nanos(in.time_limit)};
  google::protobuf::Duration *const time_limit = out->mutable_time_limit();
  time_limit->set_seconds(time_limit_secs_and_nanos.secs);
  time_limit->set_nanos(time_limit_secs_and_nanos.nanos);
  for (const experiences::Condition &condition : in.conditions) {
    Condition *const proto_condition = out->add_conditions();
    google::protobuf::Duration *const delay = proto_condition->mutable_delay();
    const time::SecsAndNanos duration_secs_and_nanos{
        time::to_seconds_and_nanos(condition.delay)};
    delay->set_seconds(duration_secs_and_nanos.secs);
    delay->set_nanos(duration_secs_and_nanos.nanos);
    pack(condition.condition, proto_condition->mutable_location_condition());
  }
}

experiences::CompletionCriteria unpack(const CompletionCriteria &in) {
  experiences::CompletionCriteria result;
  const time::Duration &time_limit{time::from_seconds_and_nanos(
      {in.time_limit().seconds(), in.time_limit().nanos()})};
  result.time_limit = time_limit;
  for (const Condition &condition_proto : in.conditions()) {
    experiences::Condition condition;
    const time::Duration &delay{time::from_seconds_and_nanos(
        {condition_proto.delay().seconds(), condition_proto.delay().nanos()})};
    condition.delay = delay;
    condition.condition = unpack(condition_proto.location_condition());
    result.conditions.emplace_back(condition);
  }
  return result;
}

}  // namespace resim::experiences::proto
