# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary", "ros2_cpp_test")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_python//python:proto.bzl", "py_proto_library")

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

py_proto_library(
    name = "header_proto_py",
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

py_proto_library(
    name = "transform_proto_py",
    deps = [
        ":transform_proto",
    ],
)

proto_library(
    name = "pose_proto",
    srcs = ["pose.proto"],
    deps = [
        "//resim/transforms/proto:se3_proto",
    ],
)

cc_proto_library(
    name = "pose_proto_cc",
    deps = [
        ":pose_proto",
    ],
)

py_proto_library(
    name = "pose_proto_py",
    deps = [
        ":pose_proto",
    ],
)

proto_library(
    name = "odometry_proto",
    srcs = ["odometry.proto"],
    deps = [
        ":header_proto",
        ":pose_proto",
    ],
)

cc_proto_library(
    name = "odometry_proto_cc",
    deps = [
        ":odometry_proto",
    ],
)

py_proto_library(
    name = "odometry_proto_py",
    deps = [
        ":odometry_proto",
    ],
)

proto_library(
    name = "detection_proto",
    srcs = ["detection.proto"],
    deps = [
        ":header_proto",
        "//resim/geometry/proto:oriented_box_proto",
    ],
)

cc_proto_library(
    name = "detection_proto_cc",
    deps = [
        ":detection_proto",
    ],
)

py_proto_library(
    name = "detection_proto_py",
    deps = [
        ":detection_proto",
    ],
)

proto_library(
    name = "navsat_proto",
    srcs = ["navsat.proto"],
    deps = [
        ":header_proto",
    ],
)

cc_proto_library(
    name = "navsat_proto_cc",
    deps = [
        ":navsat_proto",
    ],
)

py_proto_library(
    name = "navsat_proto_py",
    deps = [
        ":navsat_proto",
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
        ":header_from_ros2",
        ":transform_proto_cc",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/transforms/proto:se3_proto_cc",
        "//resim/transforms/proto:se3_to_proto",
        "@libeigen//:eigen",
        "@ros2_common_interfaces//:cpp_geometry_msgs",
        "@ros2_geometry2//:cpp_tf2_msgs",
    ],
)

cc_test(
    name = "transform_from_ros2_test",
    srcs = ["transform_from_ros2_test.cc"],
    deps = [
        ":fuzz_helpers",
        ":transform_from_ros2",
        ":transform_proto_cc",
        "//resim/testing:fuzz_helpers",
        "//resim/utils:inout",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "pose_from_ros2",
    srcs = ["pose_from_ros2.cc"],
    hdrs = ["pose_from_ros2.hh"],
    deps = [
        ":pose_proto_cc",
        "//resim/assert",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/transforms/proto:se3_proto_cc",
        "//resim/transforms/proto:se3_to_proto",
        "@libeigen//:eigen",
        "@ros2_common_interfaces//:cpp_geometry_msgs",
    ],
)

cc_test(
    name = "pose_from_ros2_test",
    srcs = ["pose_from_ros2_test.cc"],
    deps = [
        ":fuzz_helpers",
        ":pose_from_ros2",
        "//resim/testing:fuzz_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "odometry_from_ros2",
    srcs = ["odometry_from_ros2.cc"],
    hdrs = ["odometry_from_ros2.hh"],
    deps = [
        ":header_from_ros2",
        ":odometry_proto_cc",
        ":pose_from_ros2",
        "@ros2_common_interfaces//:cpp_nav_msgs",
    ],
)

cc_test(
    name = "odometry_from_ros2_test",
    srcs = ["odometry_from_ros2_test.cc"],
    deps = [
        ":fuzz_helpers",
        ":odometry_from_ros2",
        "//resim/testing:fuzz_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "oriented_box_from_ros2",
    srcs = ["oriented_box_from_ros2.cc"],
    hdrs = ["oriented_box_from_ros2.hh"],
    deps = [
        ":pose_from_ros2",
        "//resim/assert",
        "//resim/geometry/proto:oriented_box_proto_cc",
        "@vision_msgs//:cpp_vision_msgs",
    ],
)

cc_test(
    name = "oriented_box_from_ros2_test",
    srcs = ["oriented_box_from_ros2_test.cc"],
    deps = [
        ":oriented_box_from_ros2",
        "//resim/geometry/proto:fuzz_helpers",
        "//resim/testing:fuzz_helpers",
        "//resim/transforms:frame",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "detection_from_ros2",
    srcs = ["detection_from_ros2.cc"],
    hdrs = ["detection_from_ros2.hh"],
    deps = [
        ":detection_proto_cc",
        ":header_from_ros2",
        ":oriented_box_from_ros2",
        "@vision_msgs//:cpp_vision_msgs",
    ],
)

cc_test(
    name = "detection_from_ros2_test",
    srcs = ["detection_from_ros2_test.cc"],
    deps = [
        ":detection_from_ros2",
        ":fuzz_helpers",
        "//resim/testing:fuzz_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "navsat_from_ros2",
    srcs = ["navsat_from_ros2.cc"],
    hdrs = ["navsat_from_ros2.hh"],
    deps = [
        ":header_from_ros2",
        ":navsat_proto_cc",
        "//resim/assert",
        "@ros2_common_interfaces//:cpp_sensor_msgs",
    ],
)

cc_test(
    name = "navsat_from_ros2_test",
    srcs = ["navsat_from_ros2_test.cc"],
    deps = [
        ":fuzz_helpers",
        ":navsat_from_ros2",
        "//resim/testing:fuzz_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "converter_plugin_types",
    hdrs = ["converter_plugin_types.h"],
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
        ":converter_plugin_types",
        "//resim/assert",
        "@com_google_googletest//:gtest_prod",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_test(
    name = "converter_plugin_test",
    srcs = ["converter_plugin_test.cc"],
    data = [
        "//resim/msg/testing:good_test_converter_plugin.so",
    ],
    deps = [
        ":converter_plugin",
        "//resim/assert",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "default_converter_plugin.so",
    srcs = ["default_converter_plugin.cc"],
    linkshared = True,
    visibility = ["//resim/msg:__subpackages__"],
    deps = [
        ":detection_from_ros2",
        ":header_from_ros2",
        ":navsat_from_ros2",
        ":odometry_from_ros2",
        ":oriented_box_from_ros2",
        ":pose_from_ros2",
        ":time_from_ros2",
        ":transform_from_ros2",
        "//resim/assert",
        "//resim/msg:converter_plugin_types",
        "//resim/utils/proto:dependency_file_descriptor_set",
        "@ros2_rclcpp//:rclcpp",
    ],
)

cc_test(
    name = "default_converter_plugin_test",
    srcs = ["default_converter_plugin_test.cc"],
    data = [":default_converter_plugin.so"],
    deps = [
        ":converter_plugin",
        ":detection_from_ros2",
        ":fuzz_helpers",
        ":header_from_ros2",
        ":navsat_from_ros2",
        ":odometry_from_ros2",
        ":odometry_proto_cc",
        ":oriented_box_from_ros2",
        ":pose_from_ros2",
        ":time_from_ros2",
        ":transform_from_ros2",
        ":transform_proto_cc",
        "//resim/assert",
        "//resim/utils:inout",
        "//resim/utils/proto:dependency_file_descriptor_set",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "resim_log_from_ros2",
    srcs = ["resim_log_from_ros2.cc"],
    hdrs = ["resim_log_from_ros2.hh"],
    deps = [
        ":converter_plugin",
        "//resim/assert",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/utils:inout",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rosbag2//:rosbag2_cpp",
    ],
)

ros2_cpp_test(
    name = "resim_log_from_ros2_test",
    testonly = True,  # Needed because ros2_cpp_test generates a non-test rule
    srcs = ["resim_log_from_ros2_test.cc"],
    data = [":default_converter_plugin.so"],
    set_up_ament = True,
    deps = [
        ":converter_plugin",
        ":resim_log_from_ros2",
        "//resim/msg:fuzz_helpers",
        "//resim/msg:odometry_from_ros2",
        "//resim/msg:transform_from_ros2",
        "//resim/testing:fuzz_helpers",
        "//resim/testing:test_directory",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/time:timestamp",
        "//resim/utils:inout",
        "//resim/utils:tuple_utils",
        "//resim/utils/proto:dependency_file_descriptor_set",
        "@com_google_googletest//:gtest_main",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rosbag2//:rosbag2_cpp",
        "@ros2_rosbag2//:rosbag2_storage",
    ],
)

ros2_cpp_binary(
    name = "convert_log",
    srcs = ["convert_log.cc"],
    data = [":default_converter_plugin.so"],
    set_up_ament = True,
    visibility = ["//visibility:public"],
    deps = [
        ":converter_plugin",
        ":resim_log_from_ros2",
        "//resim/assert",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/utils:inout",
        "@cxxopts",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rosbag2//:rosbag2_cpp",
    ],
)

################################################################################
# Helpers
################################################################################

cc_library(
    name = "fuzz_helpers",
    testonly = True,
    srcs = ["fuzz_helpers.cc"],
    hdrs = ["fuzz_helpers.hh"],
    deps = [
        ":detection_proto_cc",
        ":header_proto_cc",
        ":navsat_proto_cc",
        ":odometry_proto_cc",
        ":pose_proto_cc",
        ":transform_proto_cc",
        "//resim/geometry/proto:fuzz_helpers",
        "//resim/testing:fuzz_helpers",
        "//resim/testing:random_matrix",
        "//resim/transforms:frame",
        "//resim/transforms/proto:fuzz_helpers",
        "//resim/utils:inout",
        "//resim/utils:uuid",
    ],
)

cc_test(
    name = "fuzz_helpers_test",
    testonly = True,
    srcs = ["fuzz_helpers_test.cc"],
    deps = [
        ":fuzz_helpers",
        ":header_proto_cc",
        ":odometry_proto_cc",
        ":pose_proto_cc",
        ":transform_proto_cc",
        "//resim/geometry/proto:fuzz_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)
