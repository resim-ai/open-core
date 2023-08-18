// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/time_from_ros2.hh"

namespace resim::msg {

google::protobuf::Timestamp convert_from_ros2(
    const builtin_interfaces::msg::Time &ros2_msg) {
  google::protobuf::Timestamp stamp;
  stamp.set_seconds(ros2_msg.sec);
  stamp.set_nanos(static_cast<int>(ros2_msg.nanosec));
  return stamp;
}

builtin_interfaces::msg::Time convert_to_ros2(
    const google::protobuf::Timestamp &time) {
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int>(time.seconds());
  result.nanosec = time.nanos();
  return result;
}

}  // namespace resim::msg
