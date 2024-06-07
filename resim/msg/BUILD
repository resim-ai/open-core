# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_python//python:proto.bzl", "py_proto_library")

################################################################################
# Protos
################################################################################

proto_library(
    name = "header_proto",
    srcs = ["header.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "header_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":header_proto",
    ],
)

py_proto_library(
    name = "header_proto_py",
    visibility = ["//visibility:public"],
    deps = [
        ":header_proto",
    ],
)

proto_library(
    name = "transform_proto",
    srcs = ["transform.proto"],
    visibility = ["//visibility:public"],
    deps = [
        ":header_proto",
        "//resim/transforms/proto:se3_proto",
    ],
)

cc_proto_library(
    name = "transform_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":transform_proto",
    ],
)

py_proto_library(
    name = "transform_proto_py",
    visibility = ["//visibility:public"],
    deps = [
        ":transform_proto",
    ],
)

proto_library(
    name = "pose_proto",
    srcs = ["pose.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/transforms/proto:se3_proto",
    ],
)

cc_proto_library(
    name = "pose_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":pose_proto",
    ],
)

py_proto_library(
    name = "pose_proto_py",
    visibility = ["//visibility:public"],
    deps = [
        ":pose_proto",
    ],
)

proto_library(
    name = "odometry_proto",
    srcs = ["odometry.proto"],
    visibility = ["//visibility:public"],
    deps = [
        ":header_proto",
        ":pose_proto",
    ],
)

cc_proto_library(
    name = "odometry_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":odometry_proto",
    ],
)

py_proto_library(
    name = "odometry_proto_py",
    visibility = ["//visibility:public"],
    deps = [
        ":odometry_proto",
    ],
)

proto_library(
    name = "detection_proto",
    srcs = ["detection.proto"],
    visibility = ["//visibility:public"],
    deps = [
        ":header_proto",
        ":pose_proto",
        "//resim/geometry/proto:oriented_box_proto",
    ],
)

cc_proto_library(
    name = "detection_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":detection_proto",
    ],
)

py_proto_library(
    name = "detection_proto_py",
    visibility = ["//visibility:public"],
    deps = [
        ":detection_proto",
    ],
)

proto_library(
    name = "navsat_proto",
    srcs = ["navsat.proto"],
    visibility = ["//visibility:public"],
    deps = [
        ":header_proto",
    ],
)

cc_proto_library(
    name = "navsat_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":navsat_proto",
    ],
)

py_proto_library(
    name = "navsat_proto_py",
    visibility = ["//visibility:public"],
    deps = [
        ":navsat_proto",
    ],
)

proto_library(
    name = "primitives_proto",
    srcs = ["primitives.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "primitives_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":primitives_proto",
    ],
)

py_proto_library(
    name = "primitives_proto_py",
    visibility = ["//visibility:public"],
    deps = [
        ":primitives_proto",
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
    visibility = ["//visibility:public"],
    deps = [
        ":byte_swap_helpers",
        ":detection_proto_cc",
        ":header_proto_cc",
        ":navsat_proto_cc",
        ":odometry_proto_cc",
        ":pose_proto_cc",
        ":primitives_proto_cc",
        ":transform_proto_cc",
        "//resim/converter:fuzz_helpers",
        "//resim/converter:parser",
        "//resim/geometry/proto:fuzz_helpers",
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

cc_library(
    name = "byte_swap_helpers",
    srcs = ["byte_swap_helpers.cc"],
    hdrs = ["byte_swap_helpers.hh"],
    visibility = [
        "//resim/msg:__subpackages__",
        "//resim/ros2:__subpackages__",
    ],
    deps = [
        ":primitives_proto_cc",
        "//resim/utils:inout",
    ],
)

cc_test(
    name = "byte_swap_helpers_test",
    srcs = ["byte_swap_helpers_test.cc"],
    deps = [
        ":byte_swap_helpers",
        ":primitives_proto_cc",
        "//resim/utils:inout",
        "@com_google_googletest//:gtest_main",
    ],
)
