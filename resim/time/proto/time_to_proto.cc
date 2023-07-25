
#include "resim/time/proto/time_to_proto.hh"

#include "resim/assert/assert.hh"
#include "resim/time/timestamp.hh"

namespace resim::time::proto {

void pack(const time::Timestamp in, google::protobuf::Timestamp *const out) {
  REASSERT(out != nullptr, "Can't pack invalid timestamp!");
  out->Clear();
  const time::SecsAndNanos secs_and_nanos{
      time::to_seconds_and_nanos(in.time_since_epoch())};
  out->set_seconds(secs_and_nanos.secs);
  out->set_nanos(secs_and_nanos.nanos);
}

void pack(const time::Duration in, google::protobuf::Duration *const out) {
  REASSERT(out != nullptr, "Can't pack invalid duration!");
  out->Clear();
  const time::SecsAndNanos secs_and_nanos{time::to_seconds_and_nanos(in)};
  out->set_seconds(secs_and_nanos.secs);
  out->set_nanos(secs_and_nanos.nanos);
}

time::Timestamp unpack(const google::protobuf::Timestamp &in) {
  return time::Timestamp(time::from_seconds_and_nanos(
      SecsAndNanos{.secs = in.seconds(), .nanos = in.nanos()}));
}

time::Duration unpack(const google::protobuf::Duration &in) {
  return time::from_seconds_and_nanos(
      SecsAndNanos{.secs = in.seconds(), .nanos = in.nanos()});
}

}  // namespace resim::time::proto
