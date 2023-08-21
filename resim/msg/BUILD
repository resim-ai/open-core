# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "header_proto",
    srcs = ["header.proto"],
    deps = [
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "header_proto_cc",
    deps = [
        ":header_proto",
    ],
)

cc_library(
    name = "header_from_ros2",
    srcs = ["header_from_ros2.cc"],
    hdrs = ["header_from_ros2.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":header_proto_cc",
        ":time_from_ros2",
        "@ros2_common_interfaces//:cpp_std_msgs",
    ],
)

cc_test(
    name = "header_from_ros2_test",
    srcs = ["header_from_ros2_test.cc"],
    deps = [
        ":header_from_ros2",
        "@com_google_googletest//:gtest_main",
        "@ros2_common_interfaces//:cpp_std_msgs",
    ],
)

cc_library(
    name = "time_from_ros2",
    srcs = ["time_from_ros2.cc"],
    hdrs = ["time_from_ros2.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_protobuf//:protobuf",
        "@ros2_rcl_interfaces//:cpp_builtin_interfaces",
    ],
)

cc_test(
    name = "time_from_ros2_test",
    srcs = ["time_from_ros2_test.cc"],
    deps = [
        ":time_from_ros2",
        "@com_google_googletest//:gtest_main",
        "@ros2_rcl_interfaces//:cpp_builtin_interfaces",
    ],
)

cc_library(
    name = "converter_plugin_status",
    hdrs = ["converter_plugin_status.h"],
    visibility = ["//resim/msg:__subpackages__"],
    deps = [
    ],
)

cc_library(
    name = "converter_plugin",
    srcs = ["converter_plugin.cc"],
    hdrs = ["converter_plugin.hh"],
    visibility = ["//resim/msg:__subpackages__"],
    deps = [
        ":converter_plugin_status",
        "//resim/assert",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_test(
    name = "converter_plugin_test",
    srcs = ["converter_plugin_test.cc"],
    data = [
        "//resim/msg/testing:bad_test_converter_plugin.so",
        "//resim/msg/testing:good_test_converter_plugin.so",
    ],
    deps = [
        ":converter_plugin",
        "//resim/assert",
        "@com_google_googletest//:gtest_main",
    ],
)
