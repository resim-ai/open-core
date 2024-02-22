# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https:#opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "color_to_foxglove",
    srcs = ["color_to_foxglove.cc"],
    hdrs = ["color_to_foxglove.hh"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//resim/assert",
        "//resim/visualization:color",
        "@foxglove_schemas",
    ],
)

cc_test(
    name = "color_to_foxglove_test",
    srcs = ["color_to_foxglove_test.cc"],
    deps = [
        ":color_to_foxglove",
        "@com_google_googletest//:gtest_main",
        "@foxglove_schemas",
    ],
)

cc_library(
    name = "orientation_to_foxglove",
    srcs = ["orientation_to_foxglove.cc"],
    hdrs = ["orientation_to_foxglove.hh"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//resim/assert",
        "//resim/transforms:so3",
        "@foxglove_schemas",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "orientation_to_foxglove_test",
    srcs = ["orientation_to_foxglove_test.cc"],
    deps = [
        ":orientation_to_foxglove",
        "//resim/testing:random_matrix",
        "//resim/transforms:so3",
        "@com_google_googletest//:gtest_main",
        "@foxglove_schemas",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "vector_to_foxglove",
    srcs = ["vector_to_foxglove.cc"],
    hdrs = ["vector_to_foxglove.hh"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//resim/assert",
        "@foxglove_schemas",
        "@libeigen//:eigen",
    ],
)

cc_test(
    name = "vector_to_foxglove_test",
    srcs = ["vector_to_foxglove_test.cc"],
    deps = [
        ":vector_to_foxglove",
        "//resim/testing:random_matrix",
        "@com_google_googletest//:gtest_main",
        "@foxglove_schemas",
    ],
)

cc_library(
    name = "pose_to_foxglove",
    srcs = ["pose_to_foxglove.cc"],
    hdrs = ["pose_to_foxglove.hh"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":orientation_to_foxglove",
        ":vector_to_foxglove",
        "//resim/assert",
        "//resim/transforms:se3",
        "@foxglove_schemas",
    ],
)

cc_test(
    name = "pose_to_foxglove_test",
    srcs = ["pose_to_foxglove_test.cc"],
    deps = [
        ":pose_to_foxglove",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
        "@foxglove_schemas",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "frame_transform_to_foxglove",
    srcs = ["frame_transform_to_foxglove.cc"],
    hdrs = ["frame_transform_to_foxglove.hh"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":orientation_to_foxglove",
        ":vector_to_foxglove",
        "//resim/assert",
        "//resim/time:timestamp",
        "//resim/time/proto:time_to_proto",
        "//resim/transforms:se3",
    ],
)

cc_test(
    name = "frame_transform_to_foxglove_test",
    srcs = ["frame_transform_to_foxglove_test.cc"],
    deps = [
        ":frame_transform_to_foxglove",
        ":orientation_to_foxglove",
        ":vector_to_foxglove",
        "//resim/time:timestamp",
        "//resim/transforms:liegroup_test_helpers",
        "//resim/transforms:se3",
        "@com_google_googletest//:gtest_main",
        "@foxglove_schemas",
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "wireframe_to_foxglove",
    srcs = ["wireframe_to_foxglove.cc"],
    hdrs = ["wireframe_to_foxglove.hh"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":color_to_foxglove",
        ":pose_to_foxglove",
        ":vector_to_foxglove",
        "//resim/assert",
        "//resim/geometry:wireframe",
        "//resim/transforms:se3",
        "//resim/visualization:color",
        "@foxglove_schemas",
    ],
)

cc_test(
    name = "wireframe_to_foxglove_test",
    srcs = ["wireframe_to_foxglove_test.cc"],
    deps = [
        ":wireframe_to_foxglove",
        "//resim/geometry:drone_wireframe",
        "//resim/geometry:wireframe",
        "//resim/visualization:color",
        "@com_google_googletest//:gtest_main",
        "@foxglove_schemas",
        "@libeigen//:eigen",
    ],
)
