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
