# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

################################################################################
# Protos
################################################################################

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

proto_library(
    name = "transform_proto",
    srcs = ["transform.proto"],
    deps = [
        ":header_proto",
        "//resim/transforms/proto:se3_proto",
    ],
)

cc_proto_library(
    name = "transform_proto_cc",
    deps = [
        ":transform_proto",
    ],
)

################################################################################
# Converters
################################################################################

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
    name = "transform_from_ros2",
    srcs = ["transform_from_ros2.cc"],
    hdrs = ["transform_from_ros2.hh"],
    deps = [
        ":transform_proto_cc",
        "//resim/transforms/proto:se3_to_proto",
        "@ros2_common_interfaces//:cpp_geometry_msgs",
        "@ros2_geometry2//:cpp_tf2_msgs",
    ],
)
