// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/ros2/primitives_from_ros2.hh"

#include "resim/assert/assert.hh"
#include "resim/msg/byte_swap_helpers.hh"

namespace resim::ros2 {

msg::Bool convert_from_ros2(const std_msgs::msg::Bool &ros2_msg) {
  msg::Bool result;
  result.set_data(ros2_msg.data);
  return result;
}

std_msgs::msg::Bool convert_to_ros2(const msg::Bool &resim_msg) {
  std_msgs::msg::Bool result;
  result.data = resim_msg.data();
  return result;
}

msg::Byte convert_from_ros2(const std_msgs::msg::Byte &ros2_msg) {
  msg::Byte result;
  result.set_data(std::string{static_cast<char>(ros2_msg.data)});
  return result;
}

std_msgs::msg::Byte convert_to_ros2(const msg::Byte &resim_msg) {
  std_msgs::msg::Byte result;
  REASSERT(resim_msg.data().size() == 1U);
  result.data = resim_msg.data().at(0);
  return result;
}

msg::Char convert_from_ros2(const std_msgs::msg::Char &ros2_msg) {
  msg::Char result;
  result.set_data(std::string{static_cast<char>(ros2_msg.data)});
  return result;
}

std_msgs::msg::Char convert_to_ros2(const msg::Char &resim_msg) {
  std_msgs::msg::Char result;
  REASSERT(resim_msg.data().size() == 1U);
  result.data = resim_msg.data().at(0);
  return result;
}

msg::Empty convert_from_ros2(const std_msgs::msg::Empty &ros2_msg) {
  msg::Empty result;
  return result;
}

std_msgs::msg::Empty convert_to_ros2(const msg::Empty &resim_msg) {
  std_msgs::msg::Empty result;
  return result;
}

msg::Float32 convert_from_ros2(const std_msgs::msg::Float32 &ros2_msg) {
  msg::Float32 result;
  result.set_data(ros2_msg.data);
  return result;
}

std_msgs::msg::Float32 convert_to_ros2(const msg::Float32 &resim_msg) {
  std_msgs::msg::Float32 result;
  result.data = resim_msg.data();
  return result;
}

msg::Float64 convert_from_ros2(const std_msgs::msg::Float64 &ros2_msg) {
  msg::Float64 result;
  result.set_data(ros2_msg.data);
  return result;
}

std_msgs::msg::Float64 convert_to_ros2(const msg::Float64 &resim_msg) {
  std_msgs::msg::Float64 result;
  result.data = resim_msg.data();
  return result;
}

msg::Int16 convert_from_ros2(const std_msgs::msg::Int16 &ros2_msg) {
  msg::Int16 result;
  set_data(ros2_msg.data, InOut{result});
  return result;
}

std_msgs::msg::Int16 convert_to_ros2(const msg::Int16 &resim_msg) {
  std_msgs::msg::Int16 result;
  result.data = data(resim_msg);
  return result;
}

msg::Int32 convert_from_ros2(const std_msgs::msg::Int32 &ros2_msg) {
  msg::Int32 result;
  result.set_data(ros2_msg.data);
  return result;
}

std_msgs::msg::Int32 convert_to_ros2(const msg::Int32 &resim_msg) {
  std_msgs::msg::Int32 result;
  result.data = resim_msg.data();
  return result;
}

msg::Int64 convert_from_ros2(const std_msgs::msg::Int64 &ros2_msg) {
  msg::Int64 result;
  result.set_data(ros2_msg.data);
  return result;
}

std_msgs::msg::Int64 convert_to_ros2(const msg::Int64 &resim_msg) {
  std_msgs::msg::Int64 result;
  result.data = resim_msg.data();
  return result;
}

msg::Int8 convert_from_ros2(const std_msgs::msg::Int8 &ros2_msg) {
  msg::Int8 result;
  result.set_data(std::string{ros2_msg.data});
  return result;
}

std_msgs::msg::Int8 convert_to_ros2(const msg::Int8 &resim_msg) {
  std_msgs::msg::Int8 result;
  REASSERT(resim_msg.data().size() == 1U);
  result.data = resim_msg.data().at(0);
  return result;
}

msg::String convert_from_ros2(const std_msgs::msg::String &ros2_msg) {
  msg::String result;
  result.set_data(ros2_msg.data);
  return result;
}

std_msgs::msg::String convert_to_ros2(const msg::String &resim_msg) {
  std_msgs::msg::String result;
  result.data = resim_msg.data();
  return result;
}

msg::UInt16 convert_from_ros2(const std_msgs::msg::UInt16 &ros2_msg) {
  msg::UInt16 result;
  set_data(ros2_msg.data, InOut{result});
  return result;
}

std_msgs::msg::UInt16 convert_to_ros2(const msg::UInt16 &resim_msg) {
  std_msgs::msg::UInt16 result;
  result.data = data(resim_msg);
  return result;
}

msg::UInt32 convert_from_ros2(const std_msgs::msg::UInt32 &ros2_msg) {
  msg::UInt32 result;
  result.set_data(ros2_msg.data);
  return result;
}

std_msgs::msg::UInt32 convert_to_ros2(const msg::UInt32 &resim_msg) {
  std_msgs::msg::UInt32 result;
  result.data = resim_msg.data();
  return result;
}

msg::UInt64 convert_from_ros2(const std_msgs::msg::UInt64 &ros2_msg) {
  msg::UInt64 result;
  result.set_data(ros2_msg.data);
  return result;
}

std_msgs::msg::UInt64 convert_to_ros2(const msg::UInt64 &resim_msg) {
  std_msgs::msg::UInt64 result;
  result.data = resim_msg.data();
  return result;
}

msg::UInt8 convert_from_ros2(const std_msgs::msg::UInt8 &ros2_msg) {
  msg::UInt8 result;
  result.set_data(std::string{static_cast<char>(ros2_msg.data)});
  return result;
}

std_msgs::msg::UInt8 convert_to_ros2(const msg::UInt8 &resim_msg) {
  std_msgs::msg::UInt8 result;
  REASSERT(resim_msg.data().size() == 1U);
  result.data = resim_msg.data().at(0);
  return result;
}

}  // namespace resim::ros2
