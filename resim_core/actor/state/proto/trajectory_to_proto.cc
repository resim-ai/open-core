#include "resim_core/actor/state/proto/trajectory_to_proto.hh"

#include <glog/logging.h>

#include "google/protobuf/timestamp.pb.h"
#include "resim_core/actor/state/proto/trajectory.pb.h"
#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/curves/proto/t_curve_fse3_to_proto.hh"
#include "resim_core/time/timestamp.hh"

namespace resim::actor::state::proto {

void pack(const state::Trajectory &in, Trajectory *const out) {
  CHECK(out != nullptr) << "Can't pack into invalid proto!";
  out->Clear();
  google::protobuf::Timestamp timestamp;
  // Get the start time in seconds and nanos for protobuf
  time::SecsAndNanos start_time =
      time::to_seconds_and_nanos(in.start_time().time_since_epoch());
  out->mutable_start_time()->set_nanos(start_time.nanos);
  out->mutable_start_time()->set_seconds(start_time.secs);
  pack(in.curve(), out->mutable_curve());
}

state::Trajectory unpack(const Trajectory &in) {
  time::Timestamp start_time{time::from_seconds_and_nanos(
      {in.start_time().seconds(), in.start_time().nanos()})};
  return state::Trajectory(unpack(in.curve()), start_time);
}

}  // namespace resim::actor::state::proto
