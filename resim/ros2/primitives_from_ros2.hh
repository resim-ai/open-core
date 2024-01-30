// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "resim/msg/primitives.pb.h"

namespace resim::ros2 {

// ROS2 converters for primitives

msg::Bool convert_from_ros2(const std_msgs::msg::Bool &ros2_msg);

std_msgs::msg::Bool convert_to_ros2(const msg::Bool &resim_msg);

msg::Byte convert_from_ros2(const std_msgs::msg::Byte &ros2_msg);

std_msgs::msg::Byte convert_to_ros2(const msg::Byte &resim_msg);

msg::Char convert_from_ros2(const std_msgs::msg::Char &ros2_msg);

std_msgs::msg::Char convert_to_ros2(const msg::Char &resim_msg);

msg::Empty convert_from_ros2(const std_msgs::msg::Empty &ros2_msg);

std_msgs::msg::Empty convert_to_ros2(const msg::Empty &resim_msg);

msg::Float32 convert_from_ros2(const std_msgs::msg::Float32 &ros2_msg);

std_msgs::msg::Float32 convert_to_ros2(const msg::Float32 &resim_msg);

msg::Float64 convert_from_ros2(const std_msgs::msg::Float64 &ros2_msg);

std_msgs::msg::Float64 convert_to_ros2(const msg::Float64 &resim_msg);

msg::Int16 convert_from_ros2(const std_msgs::msg::Int16 &ros2_msg);

std_msgs::msg::Int16 convert_to_ros2(const msg::Int16 &resim_msg);

msg::Int32 convert_from_ros2(const std_msgs::msg::Int32 &ros2_msg);

std_msgs::msg::Int32 convert_to_ros2(const msg::Int32 &resim_msg);

msg::Int64 convert_from_ros2(const std_msgs::msg::Int64 &ros2_msg);

std_msgs::msg::Int64 convert_to_ros2(const msg::Int64 &resim_msg);

msg::Int8 convert_from_ros2(const std_msgs::msg::Int8 &ros2_msg);

std_msgs::msg::Int8 convert_to_ros2(const msg::Int8 &resim_msg);

msg::String convert_from_ros2(const std_msgs::msg::String &ros2_msg);

std_msgs::msg::String convert_to_ros2(const msg::String &resim_msg);

msg::UInt16 convert_from_ros2(const std_msgs::msg::UInt16 &ros2_msg);

std_msgs::msg::UInt16 convert_to_ros2(const msg::UInt16 &resim_msg);

msg::UInt32 convert_from_ros2(const std_msgs::msg::UInt32 &ros2_msgz);

std_msgs::msg::UInt32 convert_to_ros2(const msg::UInt32 &resim_msg);

msg::UInt64 convert_from_ros2(const std_msgs::msg::UInt64 &ros2_msg);

std_msgs::msg::UInt64 convert_to_ros2(const msg::UInt64 &resim_msg);

msg::UInt8 convert_from_ros2(const std_msgs::msg::UInt8 &ros2_msg);

std_msgs::msg::UInt8 convert_to_ros2(const msg::UInt8 &resim_msg);

}  // namespace resim::ros2
