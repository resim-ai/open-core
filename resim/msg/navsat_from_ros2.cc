// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/navsat_from_ros2.hh"

#include "resim/assert/assert.hh"
#include "resim/msg/header_from_ros2.hh"

namespace resim::msg {

NavSatFix convert_from_ros2(const sensor_msgs::msg::NavSatFix &ros2_msg) {
  using Status = sensor_msgs::msg::NavSatStatus;
  using Fix = sensor_msgs::msg::NavSatFix;

  NavSatFix result;
  result.mutable_header()->CopyFrom(convert_from_ros2(ros2_msg.header));

  switch (ros2_msg.status.status) {
    case Status::STATUS_NO_FIX:
      result.set_status(NavSatFix::STATUS_NO_FIX);
      break;
    case Status::STATUS_FIX:
      result.set_status(NavSatFix::STATUS_FIX);
      break;
    case Status::STATUS_SBAS_FIX:
      result.set_status(NavSatFix::STATUS_SBAS_FIX);
      break;
    case Status::STATUS_GBAS_FIX:
      result.set_status(NavSatFix::STATUS_GBAS_FIX);
      break;
    default:
      REASSERT(false, "Unrecognized status!");
  }

  result.set_latitude_deg(ros2_msg.latitude);
  result.set_longitude_deg(ros2_msg.longitude);
  result.set_altitude_m(ros2_msg.altitude);

  constexpr int COV_DIM = 9;
  for (int ii = 0; ii < COV_DIM; ++ii) {
    result.add_position_covariance_m2(ros2_msg.position_covariance.at(ii));
  }

  switch (ros2_msg.position_covariance_type) {
    case Fix::COVARIANCE_TYPE_UNKNOWN:
      result.set_position_covariance_type(NavSatFix::COVARIANCE_TYPE_UNKNOWN);
      break;
    case Fix::COVARIANCE_TYPE_APPROXIMATED:
      result.set_position_covariance_type(
          NavSatFix::COVARIANCE_TYPE_APPROXIMATED);
      break;
    case Fix::COVARIANCE_TYPE_DIAGONAL_KNOWN:
      result.set_position_covariance_type(
          NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
      break;
    case Fix::COVARIANCE_TYPE_KNOWN:
      result.set_position_covariance_type(NavSatFix::COVARIANCE_TYPE_KNOWN);
      break;
    default:
      REASSERT(false, "Unrecognized covariance type!");
  }
  return result;
}

sensor_msgs::msg::NavSatFix convert_to_ros2(const NavSatFix &resim_msg) {
  using Status = sensor_msgs::msg::NavSatStatus;
  using Fix = sensor_msgs::msg::NavSatFix;

  sensor_msgs::msg::NavSatFix result;
  result.header = convert_to_ros2(resim_msg.header());

  switch (resim_msg.status()) {
    case NavSatFix::STATUS_NO_FIX:
      result.status.status = Status::STATUS_NO_FIX;
      break;
    case NavSatFix::STATUS_FIX:
      result.status.status = Status::STATUS_FIX;
      break;
    case NavSatFix::STATUS_SBAS_FIX:
      result.status.status = Status::STATUS_SBAS_FIX;
      break;
    case NavSatFix::STATUS_GBAS_FIX:
      result.status.status = Status::STATUS_GBAS_FIX;
      break;
    default:
      REASSERT(false, "Unrecognized covariance type!");
  }

  result.latitude = resim_msg.latitude_deg();
  result.longitude = resim_msg.longitude_deg();
  result.altitude = resim_msg.altitude_m();

  constexpr int COV_DIM = 9;
  REASSERT(
      resim_msg.position_covariance_m2_size() == COV_DIM,
      "Bad covariance dimension!");
  for (int ii = 0; ii < COV_DIM; ++ii) {
    result.position_covariance.at(ii) = resim_msg.position_covariance_m2(ii);
  }
  switch (resim_msg.position_covariance_type()) {
    case NavSatFix::COVARIANCE_TYPE_UNKNOWN:
      result.position_covariance_type = Fix::COVARIANCE_TYPE_UNKNOWN;
      break;
    case NavSatFix::COVARIANCE_TYPE_APPROXIMATED:
      result.position_covariance_type = Fix::COVARIANCE_TYPE_APPROXIMATED;
      break;
    case NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN:
      result.position_covariance_type = Fix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    case NavSatFix::COVARIANCE_TYPE_KNOWN:
      result.position_covariance_type = Fix::COVARIANCE_TYPE_KNOWN;
      break;
    default:
      REASSERT(false, "Unrecognized covariance type!");
  }
  return result;
}
}  // namespace resim::msg
